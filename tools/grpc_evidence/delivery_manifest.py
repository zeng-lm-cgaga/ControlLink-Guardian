#!/usr/bin/env python3
"""只读收集 P2 交付输入；每阶段 fsync，失败保留部分副本供新目录重试。

--root 指向功能仓库，--report 指向已停止写入的 P2 持久报告，--output
必须是 report 内全新的目录。脚本不构建、不执行客户端或业务判据、不提交。
manifest 只证明收集时的输入身份；旧 core/Edge 身份和失败结果不会换标。
SHA256SUMS 不包含 manifest 自身；seal.json 单向绑定 manifest 与清单。
"""

import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import stat
import subprocess
import sys


BRANCH = "codex/control-runtime-edge-integration"
BASE_COMMIT = "ceb3a3be23d6f54555630fea929497e62a3bd60e"
HISTORICAL_P2_COMMIT = "19f1c26b47b90f03643c28ed9708e1c640bff009"
EDGE_COMMIT = "394f81a8c8da544fe29eb66c80e5731fa754be7a"
EDGE_SHA256 = "70e1926fa12e3a9630ff7b2dc74d8d7b3fd62e95596be74f17f8765eb336aa05"
OWNERS = (
    "docs/00_mvp_evidence_plan.md", "docs/15_post_mvp_extensions.md",
    "docs/16_development_bug_log.md", "docs/17_network_protocol_contracts.md",
)
GENERATED_SUFFIXES = (".pb.h", ".pb.cc")
BUILD_NAMES = {"build", "CMakeFiles", "__pycache__", ".pytest_cache", ".git"}
COMMAND_SUFFIXES = (".command.json", ".command", ".exit.json", ".exit")


def git(directory, *arguments):
    """只读 Git 命令禁用可选索引锁，不触碰他人的脏工作区。"""
    environment = dict(os.environ, GIT_OPTIONAL_LOCKS="0")
    result = subprocess.run(
        ["git", "-C", str(directory), *arguments], env=environment,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False,
    )
    if result.returncode:
        raise RuntimeError(result.stderr.decode(errors="replace").strip())
    return result.stdout


def load_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def json_bytes(value):
    return (json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2,
                       allow_nan=False) + "\n").encode("utf-8")


def sync_directory(path):
    descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def write_new(path, data):
    """唯一目录中的文件也使用 exclusive create；写失败绝不伪装成功。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())
    sync_directory(path.parent)


def digest_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def file_signature(path):
    metadata = path.stat()
    if not stat.S_ISREG(metadata.st_mode):
        raise RuntimeError("Not a regular input file: " + str(path))
    return (metadata.st_dev, metadata.st_ino, metadata.st_size,
            metadata.st_mtime_ns, metadata.st_ctime_ns)


def repo_identity(root):
    return {
        "root": str(root), "branch": git(root, "branch", "--show-current").decode().strip(),
        "commit": git(root, "rev-parse", "HEAD").decode().strip(),
        "tree": git(root, "rev-parse", "HEAD^{tree}").decode().strip(),
        "tracked_status": git(root, "status", "--porcelain=v1", "--untracked-files=no").decode(),
        "untracked_status": git(root, "ls-files", "--others", "--exclude-standard", "-z").decode().split("\0")[:-1],
    }


class DeliveryCapture:
    """只拥有新交付目录；缓存输入哈希后在封存前重核，拒绝并发漂移。"""

    def __init__(self, root, report, output):
        self.root, self.report, self.output = root, report, output
        self.files = {}
        self.signatures = {}
        self.exclusions = {}

    def record(self, path, expected=None):
        path = Path(os.path.abspath(path))
        if any(character in str(path) for character in ("\n", "\r", "\\")):
            raise RuntimeError("Path cannot be represented in SHA256SUMS: " + str(path))
        resolved = path.resolve(strict=True)
        before = file_signature(resolved)
        key = str(path)
        target = os.readlink(path) if path.is_symlink() else None
        if key in self.files:
            record = self.files[key]
            if (self.signatures[key] != before or record["resolved_path"] != str(resolved)
                    or record.get("symlink_target") != target):
                raise RuntimeError("Input changed during collection: " + key)
        else:
            digest = digest_file(resolved)
            if file_signature(resolved) != before:
                raise RuntimeError("Input changed while hashing: " + key)
            with resolved.open("rb") as stream:
                magic = stream.read(8)
            record = {
                "path": key, "resolved_path": str(resolved), "bytes": before[2],
                "sha256": digest, "symlink_target": target,
                "kind": "ELF" if magic.startswith(b"\x7fELF") else
                        "static_archive" if magic == b"!<arch>\n" else "file",
            }
            self.files[key] = record
            self.signatures[key] = before
        if expected is not None and record["sha256"] != expected:
            raise RuntimeError("Recorded SHA256 mismatch: " + key)
        return dict(record)

    def copy(self, path, relative, expected=None):
        original = self.record(path, expected)
        target = self.output / relative
        write_new(target, Path(path).read_bytes())
        snapshot = self.record(target, original["sha256"])
        return {"input": original, "snapshot": snapshot}

    def stage(self, name, value):
        path = self.output / "stages" / (name + ".json")
        write_new(path, json_bytes(value))
        self.record(path)

    def exclude(self, path, reason):
        self.exclusions[str(path)] = reason

    def walk(self, directory, purpose="report"):
        """不跟随目录 symlink；明确剪枝海量依赖源码/对象，仅保留实际证据。"""
        def walk_error(error):
            raise error

        for current, directories, names in os.walk(directory, followlinks=False, onerror=walk_error):
            current = Path(current)
            for name in list(directories):
                child = current / name
                reason = None
                if child == self.output:
                    reason = "Current output excluded to avoid recursive self-hashing"
                elif child.is_symlink():
                    raise RuntimeError("Directory symlink requires an explicit input prefix: " + str(child))
                elif name in BUILD_NAMES or name.endswith("-build") or name.startswith("build-"):
                    reason = "Build/object/cache tree; selected metadata and generated code collected separately"
                elif current == self.report / "dependencies" and (
                        name.startswith(("grpc-", "abseil-cpp-", "c-ares-")) or
                        name in ("downloads", "consumer")):
                    reason = "Official dependency source/downloads; sealed blob evidence referenced separately"
                elif child != self.output and (child / "delivery-origin.json").is_file():
                    reason = "Previous delivery kept under its original manifest identity"
                if reason is not None:
                    directories.remove(name)
                    self.exclude(child, reason)
            for name in sorted(names):
                path = current / name
                if purpose == "report" and (name.endswith((".o", ".o.d", ".pyc")) or name == "CMakeCache.txt"):
                    self.exclude(path, "Duplicate compilation object/cache")
                    continue
                yield path

    def verify_unchanged(self):
        """对真实解析后的文件去重重算；过期快照不能静默混入最终清单。"""
        digests = {}
        for name, expected in self.files.items():
            path = Path(name)
            resolved = path.resolve(strict=True)
            target = os.readlink(path) if path.is_symlink() else None
            if (str(resolved) != expected["resolved_path"] or
                    target != expected["symlink_target"] or
                    file_signature(resolved) != self.signatures[name]):
                raise RuntimeError("Input changed before sealing: " + name)
            if str(resolved) not in digests:
                digests[str(resolved)] = digest_file(resolved)
            if digests[str(resolved)] != expected["sha256"]:
                raise RuntimeError("Input hash changed before sealing: " + name)


def collect_sources(capture):
    """index/HEAD/worktree blob 分开记录，脏源码不能冒充最终提交的内容。"""
    root = capture.root
    object_format = git(root, "rev-parse", "--show-object-format").decode().strip()
    heads = {}
    for entry in git(root, "ls-tree", "-rz", "HEAD").split(b"\0"):
        if entry:
            metadata, name = entry.split(b"\t", 1)
            heads[os.fsdecode(name)] = metadata.decode().split()[2]
    indexed = {}
    for entry in git(root, "ls-files", "--stage", "-z").split(b"\0"):
        if not entry:
            continue
        metadata, name = entry.split(b"\t", 1)
        mode, blob, stage = metadata.decode().split()
        if stage != "0":
            raise RuntimeError("Unmerged index entry: " + os.fsdecode(name))
        indexed[os.fsdecode(name)] = {"mode": mode, "index_blob": blob}
    untracked = [os.fsdecode(name) for name in
                 git(root, "ls-files", "--others", "--exclude-standard", "-z").split(b"\0") if name]
    sources = []
    for name in sorted(set(indexed) | set(untracked)):
        if name.startswith(("assets/", "docs/", "reports/")):
            continue
        path = root / name
        if path.is_symlink() or not path.is_file():
            raise RuntimeError("Source must be a present regular file: " + str(path))
        data = path.read_bytes()
        blob = hashlib.new(object_format, b"blob " + str(len(data)).encode() + b"\0" + data).hexdigest()
        entry = indexed.get(name, {"mode": None, "index_blob": None})
        sources.append({"path": name, **entry, "head_blob": heads.get(name),
                        "worktree_blob": blob, "tracked": name in indexed,
                        "matches_head": heads.get(name) == blob,
                        **capture.copy(path, Path("source") / name)})
    owners = [{"path": name, **capture.copy(root / name, Path("owners") / name)}
              for name in (*OWNERS, "AGENTS.md", "README.md")]
    return {"object_format": object_format, "source_inputs": sources, "owner_inputs": owners,
            "selection": "git ls-files plus explicit untracked files, excluding assets/docs/reports; docs owners separately"}


def collect_dependencies(capture):
    directory = capture.report / "dependencies"
    manifest_path = directory / "manifest.json"
    manifest = load_json(manifest_path)
    seal = load_json(directory / "seal-receipt.json")
    manifest_record = capture.record(manifest_path, seal["manifest_sha256"])
    sums_record = capture.record(directory / "SHA256SUMS", seal["sha256sums_sha256"])
    official = []
    for name, facts in sorted(manifest["source_blob_verification"].items()):
        path = (directory / facts["evidence"]).resolve(strict=True)
        if not path.is_relative_to(directory):
            raise RuntimeError("Official blob reference escaped dependency directory")
        official.append({"dependency": name, "count": facts["count"],
                         "reference": capture.record(path, facts["sha256"]),
                         "scope": "previous official blob verification, not rerun or reattributed"})
    linked = [capture.record(path, facts["sha256"])
              for path, facts in sorted(manifest["actual_linked_inputs"].items())]
    fixed = manifest["fixed_dependencies"]
    if (fixed["grpc"]["version"] != "1.30.2" or fixed["protobuf"]["version"] != "3.12.4"
            or fixed["grpc"]["commit"] != "de6defa6fff08de20e36f9168f5b277e292daf46"):
        raise RuntimeError("Dependency manifest does not describe the locked gRPC/Protobuf inputs")
    return {"manifest": manifest_record, "sha256sums": sums_record,
            "seal": capture.record(directory / "seal-receipt.json"),
            "fixed_dependencies": fixed, "official_blob_references": official,
            "historical_dependency_consumer_link_inputs": linked}


def collect_historical_source(capture, object_format):
    """git show 提取旧生产提交，不用当前工作树冒充旧场景的编译输入。"""
    revision = HISTORICAL_P2_COMMIT
    commit = git(capture.root, "rev-parse", revision + "^{commit}").decode().strip()
    tree = git(capture.root, "rev-parse", revision + "^{tree}").decode().strip()
    files = []
    for entry in git(capture.root, "ls-tree", "-rz", revision).split(b"\0"):
        if not entry:
            continue
        metadata, raw_name = entry.split(b"\t", 1)
        mode, kind, blob = metadata.decode().split()
        name = os.fsdecode(raw_name)
        if name.startswith(("assets/", "docs/", "reports/")):
            continue
        if kind != "blob" or mode not in ("100644", "100755"):
            raise RuntimeError("Unsupported historical source entry: " + name)
        data = git(capture.root, "show", revision + ":" + name)
        actual_blob = hashlib.new(object_format, b"blob " + str(len(data)).encode() + b"\0" + data).hexdigest()
        if actual_blob != blob:
            raise RuntimeError("Historical Git blob differs: " + name)
        target = capture.output / "historical-source" / revision / name
        write_new(target, data)
        snapshot = capture.record(target, hashlib.sha256(data).hexdigest())
        current = capture.record(capture.root / name) if (capture.root / name).is_file() else None
        files.append({"path": name, "mode": mode, "git_blob": blob, "snapshot": snapshot,
                      "current_source": current,
                      "matches_current_source": current["sha256"] == snapshot["sha256"] if current else False})
    scene = capture.report / "business-03-19f1c26"
    archived = [capture.record(path) for path in sorted((scene / "executed-elf").glob("*"))]
    return {"commit": commit, "tree": tree, "source_inputs": files,
            "original_scene_directory": str(scene), "archived_executed_elf": archived,
            "scope": "19f1c26 original production identity; its stored subwindows and overall failure remain original evidence, not final-new-binary acceptance"}


def collect_installations(capture, dependencies):
    environment = capture.report / "environment"
    dependency_prefix = Path(dependencies["fixed_dependencies"]["grpc"]["prefix"]).resolve(strict=True)
    prefixes = {name: environment / name for name in (
        "grpc-install", "probe-install", "consumer-install", "runtime-prefix")}
    prefixes["locked-grpc-dependency"] = dependency_prefix
    installations = []
    for name, prefix in prefixes.items():
        if not prefix.is_dir():
            raise RuntimeError("Required installation prefix missing: " + str(prefix))
        files = [capture.record(path) for path in sorted(capture.walk(prefix, "install"))]
        if not files:
            raise RuntimeError("Empty installation prefix: " + str(prefix))
        installations.append({"role": name, "prefix": str(prefix), "files": files})
    required = (
        prefixes["grpc-install"] / "bin/grpc_planning_client",
        prefixes["probe-install"] / "bin/grpc_protocol_probe",
        prefixes["consumer-install"] / "bin/grpc_planning_consumer",
        prefixes["runtime-prefix"] / "lib/control_link_adapters/grpc_planning_bridge_node",
        prefixes["runtime-prefix"] / "lib/control_link_adapters/grpc_planning_client",
    )
    for executable in required:
        if capture.record(executable)["kind"] != "ELF":
            raise RuntimeError("Installed executable is not ELF: " + str(executable))
    generated, metadata = [], []
    for build in sorted(environment.glob("*-build")):
        for current, directories, names in os.walk(build):
            current = Path(current)
            directories[:] = [name for name in directories if name not in (".git", "__pycache__")]
            for name in sorted(names):
                path = current / name
                if name.endswith(GENERATED_SUFFIXES):
                    generated.append(capture.copy(path, Path("generated") / path.relative_to(environment)))
                elif name in ("CMakeCache.txt", "link.txt", "flags.make", "compile_commands.json") or name.startswith("install_manifest"):
                    metadata.append(capture.record(path))
    cache_path = environment / "grpc-build/CMakeCache.txt"
    cache = cache_path.read_text(encoding="utf-8")
    protoc_lines = [line.split("=", 1)[1] for line in cache.splitlines()
                    if line.startswith("Protobuf_PROTOC_EXECUTABLE:FILEPATH=")]
    if len(protoc_lines) != 1:
        raise RuntimeError("Configured protoc identity missing or ambiguous")
    tools = {"protoc": capture.record(Path(protoc_lines[0])),
             "grpc_cpp_plugin": capture.record(dependency_prefix / "bin/grpc_cpp_plugin"),
             "configuration_cache": capture.record(cache_path),
             "scope": "configured generation inputs; versions read from locked manifest, no tool executed"}
    if not any(item["input"]["path"].endswith(".grpc.pb.cc") for item in generated):
        raise RuntimeError("Actual generated gRPC source missing")
    return {"installations": installations, "required_elfs": [capture.record(path) for path in required],
            "generated_inputs": generated, "build_metadata": metadata, "generation_tools": tools}


def collect_previous_inputs(capture):
    previous = capture.root / "reports/2026-10-12_p1_f1"
    manifest_path = previous / "delivery-ceb3a3b/final/manifest.json"
    old = load_json(manifest_path)
    manifest_record = capture.record(manifest_path)
    if old["source_commit"] != BASE_COMMIT:
        raise RuntimeError("P1-F1 provenance manifest identity mismatch")
    old_files = {item["path"]: item["sha256"] for section in ("binary_inputs", "persistent_artifacts")
                 for item in old.get(section, [])}
    core_prefix = previous / "environment/core-install"
    core = []
    for path in sorted(capture.walk(core_prefix, "install")):
        record = capture.record(path, old_files.get(str(path)))
        if record["kind"] == "ELF" and str(path) not in old_files:
            raise RuntimeError("Core ELF missing historical provenance: " + str(path))
        core.append({**record, "recorded_in_prior_manifest": str(path) in old_files})
    if not core:
        raise RuntimeError("Unchanged core installation missing")
    assembly_paths = sorted((capture.report / "environment").glob("*-assembly.json"))
    if not any(path.name == "runtime-assembly.json" for path in assembly_paths):
        raise RuntimeError("Runtime assembly provenance missing")
    assemblies = []
    for assembly_path in assembly_paths:
        assembly = load_json(assembly_path)
        copies = []
        for entry in assembly["copies"]:
            source = capture.record(entry["source"], entry["sha256"])
            installed = capture.record(entry["installed"], entry["sha256"])
            historical = old_files.get(entry["source"])
            copies.append({"source": source, "installed": installed,
                           "prior_manifest_sha256": historical,
                           "matches_prior_input": historical == entry["sha256"] if historical else None,
                           "scope": "exact assembly bytes; original source identity retained, not assigned a new core build commit"})
        assemblies.append({"manifest": capture.record(assembly_path),
                           "recorded_scope": assembly.get("scope"), "copies": copies})
    edge = old["dependencies"]
    if edge["edge_consumed_historical_commit"] != EDGE_COMMIT:
        raise RuntimeError("Historical Edge consumption identity mismatch")
    archive = capture.record(edge["edge_archive"]["path"], EDGE_SHA256)
    checkouts = []
    for name in ("EdgeRuntime", "EdgeRuntime-code-only-candidate-007"):
        path = capture.root.parent / name
        checkouts.append({
            "path": str(path), "head": git(path, "rev-parse", "HEAD").decode().strip(),
            "status": git(path, "status", "--porcelain=v1", "--untracked-files=all").decode(),
            "diff_name_status": git(path, "diff", "--name-status").decode(),
            "staged_diff_name_status": git(path, "diff", "--cached", "--name-status").decode(),
            "gitignore": capture.record(path / ".gitignore") if (path / ".gitignore").is_file() else None,
            "scope": "read-only snapshot; existing changes not altered or attributed to this task",
        })
    return {"prior_manifest": manifest_record, "prior_source_commit": old["source_commit"],
            "prior_base_commit": old.get("base_commit"), "core_prefix": str(core_prefix),
            "core_inputs": core, "runtime_assemblies": assemblies,
            "historical_edge_commit": EDGE_COMMIT,
            "edge_archive": archive, "edge_linked_or_rebuilt_for_p2": False,
            "edge_checkouts": checkouts,
            "scope": "P1-F1 delivery provenance retained; original core build identity is not relabeled as P2"}


def collect_commands(capture):
    directory = capture.report / "build-validation"
    groups = {}
    for path in sorted(directory.iterdir()):
        for suffix in COMMAND_SUFFIXES:
            if path.name.endswith(suffix):
                name = path.name[:-len(suffix)]
                group = groups.setdefault(name, {"name": name, "commands": [], "exits": []})
                item = {"file": capture.record(path)}
                if suffix.endswith(".json"):
                    item["value"] = load_json(path)
                else:
                    item["value"] = path.read_text(encoding="utf-8")
                if suffix.startswith(".exit"):
                    code = item["value"]["exit_code"] if isinstance(item["value"], dict) else int(item["value"].strip())
                    if type(code) is not int:
                        raise RuntimeError("Invalid command exit receipt: " + str(path))
                    item["exit_code"] = code
                    group["exits"].append(item)
                else:
                    group["commands"].append(item)
                break
    if not groups:
        raise RuntimeError("Build-validation command/exit evidence missing")
    for name, group in groups.items():
        if group["commands"] and not group["exits"]:
            raise RuntimeError("Command still lacks an exit receipt: " + name)
        group["command_record_missing"] = not group["commands"]
        log_path = directory / (name + ".log")
        group["log"] = capture.record(log_path) if log_path.is_file() else None
        group["scope"] = "original results retained, including nonzero failures; absent historical commands are not invented"
    return list(groups.values())


def process_records(value, location="events"):
    if isinstance(value, dict):
        if all(key in value for key in ("pid", "exe", "exe_sha256", "mapped_inputs")):
            yield location, value
            return
        for key, item in sorted(value.items()):
            yield from process_records(item, location + "." + key)
    elif isinstance(value, list):
        for index, item in enumerate(value):
            yield from process_records(item, location + "[" + str(index) + "]")


def recorded_runtime_input(capture, path, expected):
    """失败/旧轮的身份保留原哈希；当前文件不同不等于旧业务结论被推翻。"""
    current = capture.record(path) if Path(path).is_file() else None
    return {"recorded_path": path, "runtime_observed_sha256": expected,
            "current_file": current,
            "matches_current_file": current["sha256"] == expected if current else False,
            "persisted_matching_files": [dict(item) for name, item in sorted(capture.files.items())
                                         if item["sha256"] == expected and name != path],
            "scope": "captured runtime identity; missing/replaced historical bytes are explicit, never relabeled"}


def collect_scenes(capture, report_files):
    """只读取已有观察/冻结/重放结果，不重新运行判据或推断逐条执行 ACK。"""
    scenes = []
    for events_path in (path for path in report_files if path.name == "events.json"):
        events = load_json(events_path)
        commands_path = events_path.parent / "commands.json"
        commands = load_json(commands_path)
        if not isinstance(commands, list):
            raise RuntimeError("Scene commands must be a list: " + str(commands_path))
        processes = []
        for location, identity in process_records(events):
            executable = recorded_runtime_input(capture, identity["exe"], identity["exe_sha256"])
            mappings = []
            for path, expected in sorted(identity["mapped_inputs"].items()):
                if path.startswith("/dev/shm/fastrtps"):
                    mappings.append({"path": path, "runtime_observed_sha256": expected,
                                     "kind": "ephemeral_data_mapping", "rehash_after_exit": False})
                else:
                    mappings.append(recorded_runtime_input(capture, path, expected))
            configurations = []
            for index, (path, facts) in enumerate(sorted(identity.get("configuration_inputs", {}).items())):
                expected = facts["sha256"]
                if "utf8" not in facts:
                    configurations.append({"input": recorded_runtime_input(capture, path, expected)})
                    continue
                data = facts["utf8"].encode("utf-8")
                if hashlib.sha256(data).hexdigest() != expected or len(data) != facts["bytes"]:
                    raise RuntimeError("Runtime embedded configuration corrupted: " + path)
                target = capture.output / "runtime-config" / events_path.parent.relative_to(capture.report) / location / (str(index) + ".bin")
                write_new(target, data)
                configurations.append({"original_path": path, "snapshot": capture.record(target, expected),
                                       "live_input": recorded_runtime_input(capture, path, expected),
                                       "scope": "exact bytes captured while process lived; temporary params may be gone"})
            processes.append({"event_location": location, "recorded_pid": identity["pid"],
                              "exe": executable, "argv": identity.get("argv"),
                              "environment": identity.get("environment"), "mappings": mappings,
                              "configuration_inputs": configurations,
                              "all_code_inputs_match_current_files": executable["matches_current_file"] and all(
                                  item.get("matches_current_file", item.get("kind") == "ephemeral_data_mapping")
                                  for item in mappings),
                              "scope": "historical runtime identity, not a lookup of a possibly reused PID"})
        scenes.append({"directory": str(events_path.parent), "events": capture.record(events_path),
                       "commands": capture.record(commands_path), "recorded_commands": commands,
                       "recorded_exit_code": events.get("exit_code"), "recorded_processes": processes,
                       "recorded_period_ms": events.get("period_ms"),
                       "recorded_keep_period_ms": events.get("keep_period_ms"),
                       "observer_wait_exit": events.get("observer_wait_exit"),
                       "observer_stop_requested": events.get("observer_stop_requested"),
                       "cleanup_started": events.get("cleanup_started")})
    if not scenes:
        raise RuntimeError("No completed scene events/commands available for final delivery")
    frozen, results = [], []
    for path in report_files:
        if path.name == "manifest.json" and path.parent.name.endswith("snapshot"):
            manifest = load_json(path)
            inputs = []
            for name, facts in sorted(manifest["inputs"].items()):
                if Path(name).name != name:
                    raise RuntimeError("Frozen input path escaped snapshot: " + name)
                item = capture.record(path.parent / name, facts["sha256"])
                if item["bytes"] != facts["bytes"]:
                    raise RuntimeError("Frozen input size differs: " + name)
                inputs.append(item)
            frozen.append({"manifest": capture.record(path), "inputs": inputs,
                           "expected_gid": manifest.get("expected_gid"),
                           "waited_exit": manifest.get("waited_exit"),
                           "window": load_json(path.parent / "window.json"),
                           "receipt": load_json(path.parent / "receipt.json")})
        if path.name in ("verdict.json", "verdict-replay.json", "business-summary.json", "arbitration.json"):
            result = {"input": capture.record(path), "recorded_result": load_json(path)}
            if path.name == "verdict.json":
                replay = path.with_name("verdict-replay.json")
                result["replay"] = capture.record(replay)
                result["stored_replay_byte_identical"] = path.read_bytes() == replay.read_bytes()
                if not result["stored_replay_byte_identical"]:
                    raise RuntimeError("Stored immutable replay differs: " + str(path))
            results.append(result)
    if not frozen:
        raise RuntimeError("No frozen business input snapshot present")
    return {"scenes": scenes, "frozen_inputs": frozen, "stored_results": results,
            "replay_executed_by_generator": False,
            "scope": "stored results only; no production acceptance declared by delivery generator"}


def build_delivery(root, report, output):
    initial = repo_identity(root)
    if initial["branch"] != BRANCH:
        raise RuntimeError("Unexpected function branch: " + initial["branch"])
    output.mkdir(parents=True, exist_ok=False)
    sync_directory(output.parent)
    capture = DeliveryCapture(root, report, output)
    write_new(output / "delivery-origin.json", json_bytes({"generator": "grpc_evidence/delivery_manifest.py",
              "root": str(root), "report": str(report), "repository": initial,
              "scope": "recoverable partial output; only seal.json denotes a completed collection"}))
    try:
        source = collect_sources(capture)
        historical = collect_historical_source(capture, source["object_format"])
        capture.stage("01-source", {"repository": initial, "sources": len(source["source_inputs"]),
                                    "owners": len(source["owner_inputs"]),
                                    "historical_production_commit": historical["commit"]})
        dependencies = collect_dependencies(capture)
        installations = collect_installations(capture, dependencies)
        previous = collect_previous_inputs(capture)
        capture.stage("02-installations", {"dependencies": dependencies["manifest"],
                      "required_elfs": installations["required_elfs"],
                      "prior_manifest": previous["prior_manifest"]})
        report_files = sorted(capture.walk(report))
        persistent = [capture.record(path) for path in report_files]
        commands = collect_commands(capture)
        scenes = collect_scenes(capture, report_files)
        capture.stage("03-evidence", {"persistent_files": len(persistent), "command_groups": len(commands),
                      "scene_count": len(scenes["scenes"]), "frozen_count": len(scenes["frozen_inputs"])})
        final = repo_identity(root)
        if initial != final:
            raise RuntimeError("Repository changed during collection; retry with a new output directory")
        capture.record(output / "delivery-origin.json")
        capture.verify_unchanged()
        if report_files != sorted(capture.walk(report)):
            raise RuntimeError("Report file inventory changed during collection")
        evidence = [capture.files[name] for name in sorted(capture.files)]
        evidence_path = output / "evidence-files.json"
        write_new(evidence_path, json_bytes(evidence))
        evidence_record = capture.record(evidence_path)
        sums_path = output / "SHA256SUMS"
        write_new(sums_path, "".join(item["sha256"] + "  " + item["path"] + "\n"
                                    for item in [*evidence, evidence_record]).encode("utf-8"))
        sums_record = capture.record(sums_path)
        manifest = {
            "schema_version": 1, "stage": "P2_GRPC", "status": "PENDING_INDEPENDENT_REVIEW",
            "sealed_at_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
            "root": str(root), "report": str(report), "output": str(output), "repository": final,
            "source_commit_is_clean": not final["tracked_status"] and not final["untracked_status"],
            "scope": "VM_ONLY delivery integrity; no build/network/ROS/vcan/replay executed by this generator",
            "source": source, "historical_production_source": historical,
            "dependencies": dependencies, "installation_inputs": installations,
            "reused_inputs": previous, "build_validation": commands, "scene_evidence": scenes,
            "persistent_artifacts": persistent, "evidence_index": evidence_record, "sha256sums": sums_record,
            "excluded_paths": [{"path": path, "reason": reason} for path, reason in sorted(capture.exclusions.items())],
            "exclusion_policy": "No official dependency sources/builds/download duplication; selected generated code/build metadata explicit; all install files retained; manifest never hashes itself",
            "clock_responsibility": "wall timestamps associate SYSTEM/ROS/CAN observations; monotonic timestamps bound local waits/windows; copied as recorded and never subtracted across domains",
            "status_and_token_redaction": "Source logs and protobuf/API redaction facts retained unchanged; generator does not invent status, modify logs, or grant BUSY/readonly callers a token",
            "ignore_policy": "unchanged; ignored docs/reports remain persistent outside Git; no force-add",
            "acceptance": "Original failures remain visible. Receipt collection does not grant independent acceptance, certify safety, or create per-command execution ACKs.",
        }
        manifest_path = output / "manifest.json"
        write_new(manifest_path, json_bytes(manifest))
        write_new(output / "seal.json", json_bytes({
            "manifest_path": str(manifest_path), "manifest_sha256": digest_file(manifest_path),
            "sha256sums_path": str(sums_path), "sha256sums_sha256": sums_record["sha256"],
            "evidence_index_sha256": evidence_record["sha256"], "evidence_files": len(evidence),
            "collection_complete": True, "business_independently_accepted": False,
        }))
        sync_directory(output)
        print(json.dumps({"manifest": str(manifest_path), "sha256": digest_file(manifest_path),
                          "evidence_files": len(evidence), "status": "PENDING_INDEPENDENT_REVIEW"}, ensure_ascii=False))
    except BaseException as error:
        try:
            write_new(output / "failure.json", json_bytes({"collection_complete": False,
                      "error_type": type(error).__name__, "error": str(error),
                      "recovery": "Keep partial output; fix inputs and use a different new --output. Do not overwrite this directory."}))
        except OSError as recording_error:
            print("Failed to persist failure receipt: " + str(recording_error), file=sys.stderr)
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", required=True, type=Path)
    parser.add_argument("--report", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    arguments = parser.parse_args()
    try:
        root = arguments.root.resolve(strict=True)
        report = arguments.report.resolve(strict=True)
        output = arguments.output.resolve(strict=False)
        if (not report.is_dir() or not report.is_relative_to(root / "reports") or
                not output.is_relative_to(report) or output == report or
                output.relative_to(report).parts[0] in ("dependencies", "environment")):
            raise RuntimeError("Require --report under root/reports and a new --output under report, outside dependency/install trees")
        if os.path.lexists(arguments.output) or output.exists():
            raise FileExistsError("Refusing to overwrite output: " + str(output))
        build_delivery(root, report, output)
        return 0
    except (OSError, ValueError, KeyError, TypeError, RuntimeError, subprocess.SubprocessError) as error:
        print(type(error).__name__ + ": " + str(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
