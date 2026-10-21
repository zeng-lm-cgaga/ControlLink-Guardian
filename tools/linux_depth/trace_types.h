#ifndef CONTROL_LINK_LINUX_TRACE_TYPES_H
#define CONTROL_LINK_LINUX_TRACE_TYPES_H

struct trace_config {
	unsigned int tgid, tid;
	unsigned long long start_min_ns, start_max_ns;
};
struct cl_trace_event {
	unsigned long long at_ns, duration_ns, auxiliary_ns;
	unsigned long long task_start_ns;
	long long value;
	unsigned int tgid, tid, cpu, kind;
};
struct trace_stats {
	unsigned long long emitted, ring_full, map_full, reused_tid, cold_switch, unmatched_syscall;
};
#endif
