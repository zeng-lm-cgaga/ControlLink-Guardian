#include "vmlinux.h"
#include <bpf/bpf_helpers.h>
#include <bpf/bpf_core_read.h>
#include "trace_types.h"

char LICENSE[] SEC("license") = "GPL";
struct thread_state {
	__u64 start, wake, out, in, call;
	__s64 syscall;
};
struct { __uint(type, BPF_MAP_TYPE_ARRAY); __uint(max_entries, 1);
	__type(key, __u32); __type(value, struct trace_config); } cl_config SEC(".maps");
struct { __uint(type, BPF_MAP_TYPE_HASH); __uint(max_entries, 4096);
	__type(key, __u32); __type(value, struct thread_state); } threads SEC(".maps");
struct { __uint(type, BPF_MAP_TYPE_PERCPU_ARRAY); __uint(max_entries, 1);
	__type(key, __u32); __type(value, struct trace_stats); } stats SEC(".maps");
struct { __uint(type, BPF_MAP_TYPE_RINGBUF); __uint(max_entries, 8 * 1024 * 1024); } events SEC(".maps");

static __always_inline struct trace_stats *counters(void)
{
	__u32 key = 0;
	return bpf_map_lookup_elem(&stats, &key);
}
static __always_inline int selected(struct task_struct *task)
{
	__u32 key = 0;
	struct trace_config *cfg = bpf_map_lookup_elem(&cl_config, &key);
	if (!cfg) return 0;
	__u32 tgid = BPF_CORE_READ(task, tgid), tid = BPF_CORE_READ(task, pid);
	if (tgid != cfg->tgid || (cfg->tid && tid != cfg->tid)) return 0;
	struct task_struct *leader = BPF_CORE_READ(task, group_leader);
	__u64 start = BPF_CORE_READ(leader, start_boottime);
	return start >= cfg->start_min_ns && start < cfg->start_max_ns;
}
static __always_inline struct thread_state *state_for(struct task_struct *task)
{
	__u32 tid = BPF_CORE_READ(task, pid);
	__u64 start = BPF_CORE_READ(task, start_boottime);
	struct thread_state *state = bpf_map_lookup_elem(&threads, &tid);
	struct trace_stats *count = counters();
	if (state && state->start == start) return state;
	if (state && count) __sync_fetch_and_add(&count->reused_tid, 1);
	struct thread_state initial = { .start = start };
	if (bpf_map_update_elem(&threads, &tid, &initial, BPF_ANY)) {
		if (count) __sync_fetch_and_add(&count->map_full, 1);
		return 0;
	}
	return bpf_map_lookup_elem(&threads, &tid);
}
static __always_inline void emit(struct task_struct *task, __u32 kind, __u64 now,
	__u64 duration, __u64 auxiliary, __s64 value)
{
	struct trace_stats *count = counters();
	struct cl_trace_event *event = bpf_ringbuf_reserve(&events, sizeof(*event), 0);
	if (!event) { if (count) __sync_fetch_and_add(&count->ring_full, 1); return; }
	event->at_ns = now; event->duration_ns = duration; event->auxiliary_ns = auxiliary;
	event->task_start_ns = BPF_CORE_READ(task, start_boottime);
	event->value = value;
	event->tgid = BPF_CORE_READ(task, tgid); event->tid = BPF_CORE_READ(task, pid);
	event->cpu = bpf_get_smp_processor_id(); event->kind = kind;
	bpf_ringbuf_submit(event, 0);
	if (count) __sync_fetch_and_add(&count->emitted, 1);
}
SEC("raw_tp/sched_wakeup")
int on_wakeup(struct bpf_raw_tracepoint_args *ctx)
{
	struct task_struct *task = (void *)ctx->args[0];
	if (!selected(task)) return 0;
	struct thread_state *state = state_for(task);
	if (!state) return 0;
	__u64 now = bpf_ktime_get_ns();
	// 保存首个未消费的成功 wakeup，不用后来的通知缩短等待。
	if (!state->wake) state->wake = now;
	emit(task, 1, now, 0, 0, 0);
	return 0;
}
SEC("raw_tp/sched_switch")
int on_switch(struct bpf_raw_tracepoint_args *ctx)
{
	struct task_struct *prev = (void *)ctx->args[1], *next = (void *)ctx->args[2];
	__u64 now = bpf_ktime_get_ns();
	if (selected(prev)) {
		struct thread_state *state = state_for(prev);
		if (state) {
			emit(prev, 2, now, state->in ? now - state->in : 0, ctx->args[0], ctx->args[3]);
			state->out = now; state->in = 0;
		}
	}
	if (selected(next)) {
		struct thread_state *state = state_for(next);
		if (state) {
			struct trace_stats *count = counters();
			if (!state->out && count) __sync_fetch_and_add(&count->cold_switch, 1);
			emit(next, 3, now, state->wake ? now - state->wake : 0,
				state->out ? now - state->out : 0, state->wake ? 1 : 0);
			state->in = now; state->wake = 0; state->out = 0;
		}
	}
	return 0;
}
SEC("tracepoint/raw_syscalls/sys_enter")
int on_enter(struct trace_event_raw_sys_enter *ctx)
{
	struct task_struct *task = (void *)bpf_get_current_task();
	if (!selected(task)) return 0;
	struct thread_state *state = state_for(task);
	if (!state) return 0;
	state->call = bpf_ktime_get_ns(); state->syscall = ctx->id;
	emit(task, 4, state->call, 0, 0, ctx->id);
	return 0;
}
SEC("tracepoint/raw_syscalls/sys_exit")
int on_exit(struct trace_event_raw_sys_exit *ctx)
{
	struct task_struct *task = (void *)bpf_get_current_task();
	if (!selected(task)) return 0;
	struct thread_state *state = state_for(task);
	if (!state) return 0;
	__u64 now = bpf_ktime_get_ns();
	if (!state->call || state->syscall != ctx->id) {
		struct trace_stats *count = counters();
		if (count) __sync_fetch_and_add(&count->unmatched_syscall, 1);
		return 0;
	}
	emit(task, 5, now, now - state->call, ctx->id, ctx->ret); state->call = 0;
	return 0;
}
SEC("raw_tp/sched_process_exit")
int on_process_exit(struct bpf_raw_tracepoint_args *ctx)
{
	struct task_struct *task = (void *)ctx->args[0];
	if (!selected(task)) return 0;
	__u32 tid = BPF_CORE_READ(task, pid);
	emit(task, 6, bpf_ktime_get_ns(), 0, 0, 0);
	bpf_map_delete_elem(&threads, &tid);
	return 0;
}
