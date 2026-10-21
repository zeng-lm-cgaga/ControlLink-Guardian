#define _GNU_SOURCE
#include <bpf/libbpf.h>
#include <bpf/bpf.h>
#include <bpf/btf.h>
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "trace_types.h"

static volatile sig_atomic_t stopping;
static void stop_signal(int number) { (void)number; stopping = 1; }
static unsigned long long mono_ns(void)
{
	struct timespec now;
	if (clock_gettime(CLOCK_MONOTONIC, &now)) return 0;
	return (unsigned long long)now.tv_sec * 1000000000ULL + now.tv_nsec;
}
static unsigned long long start_ticks(unsigned int pid)
{
	char path[64], buffer[4096];
	snprintf(path, sizeof(path), "/proc/%u/stat", pid);
	FILE *input = fopen(path, "r");
	if (!input) return 0;
	char *read = fgets(buffer, sizeof(buffer), input);
	fclose(input);
	if (!read) return 0;
	char *cursor = strrchr(buffer, ')');
	if (!cursor) return 0;
	char *save = NULL, *field = strtok_r(cursor + 2, " ", &save);
	for (int index = 3; field && index < 22; ++index) field = strtok_r(NULL, " ", &save);
	return field ? strtoull(field, NULL, 10) : 0;
}
struct consumer { FILE *output; unsigned long long received; int error; };
static int consume(void *context, void *bytes, size_t size)
{
	struct consumer *sink = context;
	if (size != sizeof(struct cl_trace_event)) { sink->error = 1; return -EINVAL; }
	const struct cl_trace_event *event = bytes;
	if (fprintf(sink->output, "%u,%u,%u,%u,%llu,%llu,%llu,%llu,%lld\n",
		event->kind, event->tgid, event->tid, event->cpu, event->task_start_ns,
		event->at_ns, event->duration_ns, event->auxiliary_ns, event->value) < 0) {
		sink->error = 1; return -EIO;
	}
	++sink->received;
	return 0;
}
static unsigned long parse(const char *text, unsigned long maximum)
{
	char *end = NULL;
	errno = 0;
	unsigned long value = strtoul(text, &end, 10);
	if (errno || !end || *end || !value || value > maximum) return 0;
	return value;
}
int main(int argc, char **argv)
{
	if (argc == 3 && !strcmp(argv[1], "--check-btf")) {
		struct btf *types = btf__parse(argv[2], NULL);
		long error = libbpf_get_error(types);
		printf("{\"libbpf_version\":\"%s\",\"btf_parse_error\":%ld,\"btf_type_count\":%u}\n",
			libbpf_version_string(), error, error ? 0 : btf__type_cnt(types));
		if (!error) btf__free(types);
		return error ? 1 : 0;
	}
	if (argc < 5 || argc > 6) {
		fprintf(stderr, "usage: %s OBJECT TGID SECONDS(1..300) OUTPUT.csv [TID]\n", argv[0]); return 2;
	}
	unsigned int tgid = parse(argv[2], 2147483647), tid = argc == 6 ? parse(argv[5], 2147483647) : 0;
	unsigned int seconds = parse(argv[3], 300);
	unsigned long long initial = start_ticks(tgid);
	long ticks = sysconf(_SC_CLK_TCK);
	if (!tgid || !seconds || !initial || ticks <= 0 || (argc == 6 && !tid)) return 2;
	struct consumer sink = { .output = fopen(argv[4], "wx") };
	if (!sink.output) { perror("exclusive output"); return 2; }
	setvbuf(sink.output, NULL, _IOFBF, 1024 * 1024);
	fprintf(sink.output, "# monotonic_ns target_tgid=%u start_ticks=%llu hz=%ld target_tid=%u ring_bytes=8388608 thread_capacity=4096\n", tgid, initial, ticks, tid);
	fprintf(sink.output, "kind,tgid,tid,cpu,task_start_boottime_ns,at_ns,duration_ns,auxiliary_ns,value\n");
	struct bpf_object *object = NULL;
	struct bpf_link *links[8] = {0};
	size_t attached = 0;
	struct ring_buffer *ring = NULL;
	int result = 1;
	object = bpf_object__open_file(argv[1], NULL);
	if (libbpf_get_error(object)) { object = NULL; fprintf(stderr, "BPF open failed\n"); goto cleanup; }
	if (bpf_object__load(object)) { fprintf(stderr, "BPF load failed; no permission/verifier acceptance is claimed\n"); goto cleanup; }
	unsigned int key = 0;
	struct trace_config config = {.tgid = tgid, .tid = tid,
		.start_min_ns = initial * 1000000000ULL / ticks,
		.start_max_ns = (initial + 1) * 1000000000ULL / ticks};
	if (bpf_map_update_elem(bpf_object__find_map_fd_by_name(object, "cl_config"), &key, &config, BPF_ANY)) goto cleanup;
	struct bpf_program *program;
	bpf_object__for_each_program(program, object) {
		if (attached == sizeof(links) / sizeof(links[0])) goto cleanup;
		struct bpf_link *link = bpf_program__attach(program);
		if (libbpf_get_error(link)) { fprintf(stderr, "attach failed: %s\n", bpf_program__name(program)); goto cleanup; }
		links[attached++] = link;
	}
	ring = ring_buffer__new(bpf_object__find_map_fd_by_name(object, "events"), consume, &sink, NULL);
	if (!ring) goto cleanup;
	signal(SIGINT, stop_signal); signal(SIGTERM, stop_signal);
	unsigned long long started = mono_ns(), end = started + seconds * 1000000000ULL;
	fprintf(sink.output, "# attached=%zu start_monotonic_ns=%llu\n", attached, started);
	while (!stopping && mono_ns() < end) {
		int polled = ring_buffer__poll(ring, 100);
		if (polled < 0 && polled != -EINTR) goto cleanup;
		if (start_ticks(tgid) != initial) { fprintf(sink.output, "# target_exited_or_reused\n"); break; }
	}
	for (size_t index = 0; index < attached; ++index) { bpf_link__destroy(links[index]); links[index] = NULL; }
	attached = 0;
	if (ring_buffer__consume(ring) < 0) goto cleanup;
	int cpus = libbpf_num_possible_cpus();
	if (cpus <= 0) goto cleanup;
	struct trace_stats *values = calloc((size_t)cpus, sizeof(*values));
	if (!values) goto cleanup;
	struct trace_stats totals = {0};
	if (bpf_map_lookup_elem(bpf_object__find_map_fd_by_name(object, "stats"), &key, values)) { free(values); goto cleanup; }
	for (int cpu = 0; cpu < cpus; ++cpu) {
		totals.emitted += values[cpu].emitted; totals.ring_full += values[cpu].ring_full;
		totals.map_full += values[cpu].map_full; totals.reused_tid += values[cpu].reused_tid;
		totals.cold_switch += values[cpu].cold_switch; totals.unmatched_syscall += values[cpu].unmatched_syscall;
	}
	free(values);
	fprintf(sink.output, "# footer end_ns=%llu received=%llu emitted=%llu ring_full=%llu map_full=%llu reused_tid=%llu cold_switch=%llu unmatched_syscall=%llu\n",
		mono_ns(), sink.received, totals.emitted, totals.ring_full, totals.map_full,
		totals.reused_tid, totals.cold_switch, totals.unmatched_syscall);
	result = sink.error || totals.ring_full || totals.map_full || totals.emitted != sink.received ? 3 : 0;
cleanup:
	for (size_t index = 0; index < attached; ++index) bpf_link__destroy(links[index]);
	ring_buffer__free(ring); bpf_object__close(object);
	if (fflush(sink.output) || ferror(sink.output)) result = 4;
	if (fsync(fileno(sink.output))) result = 4;
	if (fclose(sink.output)) result = 4;
	fprintf(stderr, "loader_exit=%d; links/maps unpinned and closed\n", result);
	return result;
}
