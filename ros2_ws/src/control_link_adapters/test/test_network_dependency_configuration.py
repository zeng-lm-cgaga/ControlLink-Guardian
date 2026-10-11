import argparse
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile
import unittest


REPOSITORY = Path(__file__).resolve().parents[4]
PROBE = REPOSITORY / "tools" / "protocol_dependency_probe"
REPORT = None
PACKAGES = (
    "vsomeip3", "gRPC", "Protobuf", "EclipsePahoMqttCpp",
    "PahoMqttCpp", "eclipse-paho-mqtt-c",
)


class NetworkDependencyConfiguration(unittest.TestCase):
    def setUp(self):
        report_root = REPORT or Path(tempfile.mkdtemp(prefix="control-link-network-configuration-"))
        self.directory = report_root / self._testMethodName
        self.directory.mkdir(parents=True, exist_ok=False)
        self.prefix = self.directory / "synthetic-prefix"
        self.prefix.mkdir()
        self.calls = 0

    def configure(self, *arguments, success, diagnostic=None, build=None):
        self.calls += 1
        output_directory = self.directory / f"configure-{self.calls}"
        output_directory.mkdir()
        binary_directory = build or output_directory / "build"
        command = [
            "cmake", "-S", str(PROBE), "-B", str(binary_directory),
            "-DCMAKE_FIND_USE_PACKAGE_REGISTRY=FALSE",
            "-DCMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY=FALSE", *arguments,
        ]
        environment = os.environ.copy()
        environment.pop("AMENT_PREFIX_PATH", None)
        environment.pop("CMAKE_PREFIX_PATH", None)
        environment["MAKEFLAGS"] = "-j1"
        environment["CMAKE_BUILD_PARALLEL_LEVEL"] = "1"
        result = subprocess.run(command, capture_output=True, text=True, env=environment, timeout=60)
        (output_directory / "command.txt").write_text(shlex.join(command) + "\n")
        (output_directory / "stdout.log").write_text(result.stdout)
        (output_directory / "stderr.log").write_text(result.stderr)
        (output_directory / "exit.txt").write_text(str(result.returncode) + "\n")
        (output_directory / "scope.txt").write_text(
            "SYNTHETIC_CONFIGURATION_FIXTURE_ONLY; not real package consumption\n"
            if list(self.prefix.iterdir()) else "REAL_LOCAL_CONFIGURATION_CHECK\n"
        )
        if success:
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            report = binary_directory / "control_link_network_dependencies.txt"
            self.assertTrue(report.exists())
            self.assertIn("scope=DEPENDENCY_PROBE_ONLY", report.read_text())
            self.assertIn("production_protocol_targets=NONE", report.read_text())
        else:
            self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        if diagnostic:
            self.assertIn(diagnostic, " ".join((result.stdout + result.stderr).split()))
        return binary_directory, result

    def library(self, name, prefix=None):
        destination = (prefix or self.prefix) / "lib" / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text("SYNTHETIC_NOT_A_LINKABLE_LIBRARY\n")
        return destination

    def package(self, name, version, body):
        directory = self.prefix / "lib" / "cmake" / name
        directory.mkdir(parents=True, exist_ok=True)
        (directory / f"{name}Config.cmake").write_text(body)
        (directory / f"{name}ConfigVersion.cmake").write_text(
            f"set(PACKAGE_VERSION {version})\n"
            "if(PACKAGE_FIND_VERSION VERSION_EQUAL PACKAGE_VERSION)\n"
            "  set(PACKAGE_VERSION_EXACT TRUE)\n"
            "  set(PACKAGE_VERSION_COMPATIBLE TRUE)\n"
            "else()\n"
            "  set(PACKAGE_VERSION_COMPATIBLE FALSE)\n"
            "endif()\n"
        )
        return directory

    def imported(self, name, location, executable=False):
        definition = f"add_executable({name} IMPORTED)" if executable else f"add_library({name} UNKNOWN IMPORTED)"
        return definition + f"\nset_target_properties({name} PROPERTIES IMPORTED_LOCATION \"{location}\")\n"

    def someip(self, version="3.4.10", target=True, sd=True):
        library = self.library("libvsomeip3.so")
        body = self.imported("vsomeip3", library) if target else ""
        self.package("vsomeip3", version, body)
        if sd:
            self.library("libvsomeip3-sd.so")
        return f"-DCMAKE_PREFIX_PATH={self.prefix}"

    def grpc(self, target=True, plugin=True, protoc_version="3.12.4", generation=True, protobuf_version="3.12.4"):
        library = self.library("libgrpc++.so")
        plugin_path = self.prefix / "bin" / "grpc_cpp_plugin"
        plugin_path.parent.mkdir(parents=True)
        plugin_path.write_text("#!/bin/sh\nexit 0\n")
        plugin_path.chmod(0o755)
        body = self.imported("gRPC::grpc++", library) if target else ""
        if plugin:
            body += self.imported("gRPC::grpc_cpp_plugin", plugin_path, executable=True)
        self.package("gRPC", "1.30.2", body)
        protobuf_library = self.library("libprotobuf.so")
        compiler_path = self.prefix / "bin" / "protoc"
        compiler_path.write_text(
            "#!/usr/bin/env python3\n"
            "from pathlib import Path\nimport sys\n"
            f"if '--version' in sys.argv:\n    print('libprotoc {protoc_version}')\n    sys.exit(0)\n"
            + (
                "for argument in sys.argv[1:]:\n"
                "    if argument.startswith('--grpc_out='):\n"
                "        directory = Path(argument.split('=', 1)[1])\n"
                "        (directory / 'dependency_probe.grpc.pb.cc').write_text('SYNTHETIC_ONLY')\n"
                "        (directory / 'dependency_probe.pb.cc').write_text('SYNTHETIC_ONLY')\n"
                "sys.exit(0)\n"
                if generation else "sys.exit(7)\n"
            )
        )
        compiler_path.chmod(0o755)
        module_directory = self.prefix / "modules"
        module_directory.mkdir()
        (module_directory / "FindProtobuf.cmake").write_text(
            f"set(Protobuf_FOUND TRUE)\nset(Protobuf_VERSION {protobuf_version})\n"
            + self.imported("protobuf::libprotobuf", protobuf_library)
            + self.imported("protobuf::protoc", compiler_path, executable=True)
        )
        return [f"-DCMAKE_PREFIX_PATH={self.prefix}", f"-DCMAKE_MODULE_PATH={module_directory}"]

    def mqtt(self, cpp_target=True, cpp_version="1.2.0", c_version="1.3.9", mismatch=False, package="PahoMqttCpp"):
        c_library = self.library("libpaho-mqtt3a.so")
        cpp_library = self.library("libpaho-mqttpp3.so")
        self.package("eclipse-paho-mqtt-c", c_version, self.imported("eclipse-paho-mqtt-c::paho-mqtt3a", c_library))
        selected_c = self.library("libdifferent-paho.so") if mismatch else c_library
        body = self.imported("PahoMqttC::PahoMqttC", selected_c)
        if cpp_target:
            body += self.imported(f"{package}::paho-mqttpp3", cpp_library)
        self.package(package, cpp_version, body)
        return f"-DCMAKE_PREFIX_PATH={self.prefix}"

    def test_all_off_with_every_find_package_disabled(self):
        arguments = [f"-DCMAKE_DISABLE_FIND_PACKAGE_{package}=TRUE" for package in PACKAGES]
        binary, _ = self.configure(*arguments, success=True)
        cache = (binary / "CMakeCache.txt").read_text()
        for transport in ("SOMEIP", "GRPC", "MQTT"):
            self.assertIn(f"CONTROL_LINK_ENABLE_{transport}:BOOL=OFF", cache)
        self.assertFalse((binary / "CMakeFiles" / "control_link_grpc_dependency_probe").exists())

    def test_someip_disabled_find_fails_clearly(self):
        self.configure("-DCONTROL_LINK_ENABLE_SOMEIP=ON", "-DCMAKE_DISABLE_FIND_PACKAGE_vsomeip3=TRUE", success=False, diagnostic="SOMEIP requires vsomeip3")

    def test_grpc_disabled_find_fails_clearly(self):
        self.configure("-DCONTROL_LINK_ENABLE_GRPC=ON", "-DCMAKE_DISABLE_FIND_PACKAGE_gRPC=TRUE", success=False, diagnostic="GRPC requires gRPC")

    def test_mqtt_disabled_find_fails_clearly(self):
        self.configure("-DCONTROL_LINK_ENABLE_MQTT=ON", "-DCMAKE_DISABLE_FIND_PACKAGE_eclipse-paho-mqtt-c=TRUE", success=False, diagnostic="MQTT requires eclipse-paho-mqtt-c")

    def test_someip_wrong_version(self):
        self.configure(self.someip(version="3.4.9"), "-DCONTROL_LINK_ENABLE_SOMEIP=ON", success=False, diagnostic="exactly 3.4.10")

    def test_someip_missing_target(self):
        self.configure(self.someip(target=False), "-DCONTROL_LINK_ENABLE_SOMEIP=ON", success=False, diagnostic="required imported target vsomeip3 is missing")

    def test_someip_missing_service_discovery(self):
        self.configure(self.someip(sd=False), "-DCONTROL_LINK_ENABLE_SOMEIP=ON", success=False, diagnostic="Service Discovery library")

    def test_someip_stale_service_discovery_cache_is_rejected(self):
        old_sd = self.library("libvsomeip3-sd.so", self.directory / "old-prefix")
        self.configure(self.someip(sd=False), "-DCONTROL_LINK_ENABLE_SOMEIP=ON", f"-DCONTROL_LINK_VSOMEIP_SD_LIBRARY={old_sd}", success=False, diagnostic="Service Discovery library")

    def test_someip_reconfiguration_to_prefix_without_sd_fails(self):
        binary, _ = self.configure(self.someip(), "-DCONTROL_LINK_ENABLE_SOMEIP=ON", success=True)
        self.prefix = self.directory / "new-synthetic-prefix-without-sd"
        self.prefix.mkdir()
        self.configure(self.someip(sd=False), "-DCONTROL_LINK_ENABLE_SOMEIP=ON", "-Dvsomeip3_DIR:PATH=vsomeip3_DIR-NOTFOUND", success=False, diagnostic="Service Discovery library", build=binary)

    def test_someip_synthetic_success_is_probe_only(self):
        self.configure(self.someip(), "-DCONTROL_LINK_ENABLE_SOMEIP=ON", success=True, diagnostic="no production protocol targets")

    def test_grpc_missing_library_target(self):
        self.configure(*self.grpc(target=False), "-DCONTROL_LINK_ENABLE_GRPC=ON", success=False, diagnostic="required imported target gRPC::grpc++ is missing")

    def test_grpc_missing_plugin_target(self):
        self.configure(*self.grpc(plugin=False), "-DCONTROL_LINK_ENABLE_GRPC=ON", success=False, diagnostic="required imported target gRPC::grpc_cpp_plugin is missing")

    def test_grpc_protoc_version_mismatch(self):
        self.configure(*self.grpc(protoc_version="3.12.3"), "-DCONTROL_LINK_ENABLE_GRPC=ON", success=False, diagnostic="protoc/header/library version mismatch")

    def test_grpc_protobuf_version_mismatch(self):
        self.configure(*self.grpc(protobuf_version="3.12.3"), "-DCONTROL_LINK_ENABLE_GRPC=ON", success=False, diagnostic="does not match locked 3.12.4")

    def test_grpc_plugin_generation_failure(self):
        self.configure(*self.grpc(generation=False), "-DCONTROL_LINK_ENABLE_GRPC=ON", success=False, diagnostic="generation check failed")

    def test_grpc_synthetic_success_is_probe_only(self):
        self.configure(*self.grpc(), "-DCONTROL_LINK_ENABLE_GRPC=ON", success=True, diagnostic="no production protocol targets")

    def test_mqtt_missing_cpp_target(self):
        self.configure(self.mqtt(cpp_target=False), "-DCONTROL_LINK_ENABLE_MQTT=ON", success=False, diagnostic="required imported target PahoMqttCpp::paho-mqttpp3 is missing")

    def test_mqtt_wrong_cpp_version(self):
        self.configure(self.mqtt(cpp_version="1.1.0"), "-DCONTROL_LINK_ENABLE_MQTT=ON", success=False, diagnostic="exactly 1.2.0")

    def test_mqtt_wrong_c_version(self):
        self.configure(self.mqtt(c_version="1.3.8"), "-DCONTROL_LINK_ENABLE_MQTT=ON", success=False, diagnostic="exactly 1.3.9")

    def test_mqtt_mixed_c_prefix_is_rejected(self):
        self.configure(self.mqtt(mismatch=True), "-DCONTROL_LINK_ENABLE_MQTT=ON", success=False, diagnostic="does not reference the locked 1.3.9")

    def test_mqtt_official_upstream_synthetic_success_is_probe_only(self):
        self.configure(self.mqtt(), "-DCONTROL_LINK_ENABLE_MQTT=ON", success=True, diagnostic="no production protocol targets")

    def test_mqtt_legacy_package_synthetic_success_is_probe_only(self):
        self.configure(self.mqtt(package="EclipsePahoMqttCpp"), "-DCONTROL_LINK_ENABLE_MQTT=ON", success=True, diagnostic="no production protocol targets")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--report-dir", type=Path)
    arguments, unittest_arguments = parser.parse_known_args()
    REPORT = arguments.report_dir or Path(tempfile.mkdtemp(prefix="control-link-network-configuration-"))
    REPORT = REPORT.resolve()
    REPORT.mkdir(parents=True, exist_ok=True)
    (REPORT / "test-scope.json").write_text(json.dumps({
        "scope": "P0_DEPENDENCY_CONFIGURATION_ONLY",
        "synthetic_fixtures_are_not_real_dependency_consumption": True,
        "probe": str(PROBE), "source": str(Path(__file__).resolve()),
    }, indent=2) + "\n")
    unittest.main(argv=[sys.argv[0], *unittest_arguments], verbosity=2)
