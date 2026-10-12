#!/usr/bin/env python3
"""检查真实loader结果；ldd退出0本身不代表符号解析完整。"""

import argparse
import hashlib
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--grpc-prefix", required=True)
    parser.add_argument("executables", nargs="+")
    args = parser.parse_args()
    prefix = Path(args.grpc_prefix).resolve()
    for executable in args.executables:
        path = Path(executable).resolve(strict=True)
        result = subprocess.run(["ldd", "-r", str(path)], text=True, capture_output=True)
        output = result.stdout + result.stderr
        print(str(path), hashlib.sha256(path.read_bytes()).hexdigest(), flush=True)
        print(output, end="", flush=True)
        if result.returncode or "not found" in output or "undefined symbol:" in output:
            raise RuntimeError("loader resolution failed: " + str(path))
        observed = set()
        for line in output.splitlines():
            fields = line.split()
            if not fields or not fields[0].startswith(("libgrpc", "libgpr", "libabsl")):
                continue
            if len(fields) < 3 or fields[1] != "=>":
                raise RuntimeError("unresolved locked dependency: " + line)
            dependency = Path(fields[2]).resolve(strict=True)
            if not dependency.is_relative_to(prefix):
                raise RuntimeError("wrong locked dependency prefix: " + str(dependency))
            observed.add(fields[0])
            print("locked-input", dependency, hashlib.sha256(dependency.read_bytes()).hexdigest())
        if not any(name.startswith("libgrpc++.") for name in observed):
            raise RuntimeError("locked grpc++ input missing: " + str(path))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
