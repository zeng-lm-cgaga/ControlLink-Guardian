import json
import os
import re
import subprocess
from xml.etree import ElementTree

import yaml
from rclpy.exceptions import InvalidTopicNameException
from rclpy.validate_full_topic_name import validate_full_topic_name


def canonical_regular_file_within_root(candidate, root_path, field_name):
	canonical_root = os.path.realpath(root_path)
	if not os.path.isdir(canonical_root):
		raise RuntimeError(
			field_name + ": invalid root; expected=existing directory; actual=" +
			canonical_root)

	canonical_candidate = os.path.realpath(candidate)
	if not os.path.isfile(canonical_candidate):
		raise RuntimeError(
			field_name + ": invalid file; expected=existing regular file; actual=" +
			canonical_candidate)

	try:
		within_root = os.path.commonpath(
			[canonical_root, canonical_candidate]) == canonical_root
	except ValueError as error:
		raise RuntimeError(
			field_name + ": cannot compare paths; actual=" + str(error)) from error

	if not within_root:
		raise RuntimeError(
			field_name + ": path escapes root; expected=file within " +
			canonical_root + "; actual=" + canonical_candidate)

	return canonical_candidate


def load_yaml_mapping(file_path, field_name):
	try:
		with open(file_path, "r", encoding="utf-8") as yaml_file:
			document = yaml.safe_load(yaml_file)
	except (OSError, yaml.YAMLError) as error:
		raise RuntimeError(
			field_name + ": cannot parse YAML; actual=" + str(error)) from error

	if not isinstance(document, dict):
		raise RuntimeError(field_name + ": wrong YAML type; expected=map")
	return document


def load_record_topics(profile, profile_path):
	topics = profile.get("record_topics")
	if not isinstance(topics, list) or not topics:
		raise RuntimeError(
			profile_path +
			":record_topics: wrong YAML value; expected=non-empty sequence")

	result = []
	seen = set()
	for index, topic in enumerate(topics):
		field_path = profile_path + ":record_topics[" + str(index) + "]"
		if not isinstance(topic, str) or not topic:
			raise RuntimeError(
				field_path +
				": wrong YAML value; expected=non-empty absolute ROS Topic name")
		try:
			validate_full_topic_name(topic)
		except InvalidTopicNameException as error:
			raise RuntimeError(
				field_path + ": invalid ROS Topic name; actual=" + topic +
				"; reason=" + str(error)) from error
		if topic in seen:
			raise RuntimeError(
				field_path + ": duplicate record Topic; actual=" + topic)
		seen.add(topic)
		result.append(topic)
	return result


def load_adas_replay_config(profile, profile_path):
	if profile.get("profile_id") != "adas":
		raise RuntimeError(
			"profile_id mismatch; expected=adas; actual=" +
			str(profile.get("profile_id")))

	replay = profile.get("replay")
	if not isinstance(replay, dict):
		raise RuntimeError(profile_path + ":replay: wrong YAML type; expected=map")
	expected_fields = {
		"input_namespace",
	}
	unknown_fields = sorted(set(replay) - expected_fields)
	if unknown_fields:
		raise RuntimeError(
			profile_path + ":replay: unknown field; actual=" + unknown_fields[0])
	input_namespace = replay.get("input_namespace")
	if not isinstance(input_namespace, str) or not input_namespace:
		raise RuntimeError(
			profile_path +
			":replay.input_namespace: expected=non-empty absolute ROS namespace")
	try:
		validate_full_topic_name(input_namespace)
	except InvalidTopicNameException as error:
		raise RuntimeError(
			profile_path + ":replay.input_namespace: invalid ROS namespace; actual=" +
			input_namespace + "; reason=" + str(error)) from error
	return {"input_namespace": input_namespace}


def require_up_vcan_interface(interface):
	if re.fullmatch(r"vcan[0-9]+", interface) is None:
		raise RuntimeError(
			"ADAS execution is restricted to a vcanN interface; actual=" + interface)
	try:
		result = subprocess.run(
			["ip", "-details", "-json", "link", "show", "dev", interface],
			check=True,
			capture_output=True,
			text=True,
			timeout=3.0,
		)
		links = json.loads(result.stdout)
	except (OSError, subprocess.SubprocessError, json.JSONDecodeError) as error:
		raise RuntimeError(
			"cannot inspect ADAS CAN interface; actual=" + interface) from error
	if not isinstance(links, list) or len(links) != 1 or not isinstance(links[0], dict):
		raise RuntimeError(
			"unexpected ip link response for ADAS CAN interface; actual=" + interface)
	link = links[0]
	link_info = link.get("linkinfo")
	if not isinstance(link_info, dict) or link_info.get("info_kind") != "vcan":
		raise RuntimeError(
			"ADAS CAN interface is not an actual vcan link; actual=" + interface)
	flags = link.get("flags")
	if not isinstance(flags, list) or "UP" not in flags:
		raise RuntimeError("ADAS CAN interface must be UP; actual=" + interface)


def _source_package_root(candidate, package_name, field_path):
	current = os.path.dirname(candidate)
	while True:
		package_xml = os.path.join(current, "package.xml")
		if os.path.isfile(package_xml):
			try:
				root = ElementTree.parse(package_xml).getroot()
			except ElementTree.ParseError as error:
				raise RuntimeError(
					field_path + ": malformed source package.xml; actual=" +
					str(error)) from error
			name_node = root.find("name")
			if name_node is not None and name_node.text == package_name:
				return current

		parent = os.path.dirname(current)
		if parent == current:
			break
		current = parent

	raise RuntimeError(
		field_path + ": symlink target is not owned by package " + package_name)


def package_resource(
	package_name,
	package_share,
	reference,
	field_path,
	additional_symlink_roots=(),
):
	if not isinstance(reference, str) or not reference:
		raise RuntimeError(
			field_path + ": wrong YAML value; expected=non-empty relative path")
	if os.path.isabs(reference):
		raise RuntimeError(
			field_path + ": absolute package resource; expected=relative path; actual=" +
			reference)
	parts = reference.replace("\\", "/").split("/")
	if any(part in ("", ".", "..") for part in parts):
		raise RuntimeError(
			field_path + ": invalid package resource; expected=normalized relative path; actual=" +
			reference)

	lexical_candidate = os.path.abspath(os.path.join(package_share, *parts))
	if os.path.commonpath([os.path.abspath(package_share), lexical_candidate]) != \
		os.path.abspath(package_share):
		raise RuntimeError(
			field_path + ": package resource escapes package share; actual=" + reference)

	if os.path.islink(lexical_candidate):
		canonical_candidate = os.path.realpath(lexical_candidate)
		resource_root = None
		for candidate_root in additional_symlink_roots:
			canonical_root = os.path.realpath(candidate_root)
			if os.path.isdir(canonical_root) and os.path.commonpath(
				[canonical_root, canonical_candidate]) == canonical_root:
				resource_root = canonical_root
				break
		if resource_root is None:
			resource_root = _source_package_root(
				canonical_candidate, package_name, field_path)
	else:
		resource_root = package_share
	return canonical_regular_file_within_root(
		lexical_candidate,
		resource_root,
		field_path)


def load_fastdds_profile_path(profile_path, config_root):
	try:
		with open(profile_path, "r", encoding="utf-8") as profile_file:
			document = yaml.compose(profile_file, Loader=yaml.SafeLoader)
	except (OSError, yaml.YAMLError) as error:
		raise RuntimeError(
			profile_path + ": cannot parse Profile bootstrap YAML; actual=" +
			str(error)) from error

	if not isinstance(document, yaml.MappingNode):
		raise RuntimeError(profile_path + ": root: wrong YAML type; expected=map")

	seen_keys = set()
	fastdds_node = None
	for key_node, value_node in document.value:
		if not isinstance(key_node, yaml.ScalarNode):
			raise RuntimeError(profile_path + ": root: non-scalar YAML key")

		key = key_node.value
		if key in seen_keys:
			raise RuntimeError(
				profile_path + ":" + key +
				": duplicate field; expected=unique field")
		seen_keys.add(key)

		if key == "fastdds_profile":
			fastdds_node = value_node

	if not isinstance(fastdds_node, yaml.ScalarNode) or \
		fastdds_node.tag != "tag:yaml.org,2002:str":
		raise RuntimeError(
			profile_path +
			":fastdds_profile: wrong YAML type; expected=non-empty relative string")

	reference = fastdds_node.value
	if not reference.strip():
		raise RuntimeError(
			profile_path +
			":fastdds_profile: empty config reference; expected=non-empty relative path")
	if os.path.isabs(reference):
		raise RuntimeError(
			profile_path +
			":fastdds_profile: absolute config reference; expected=relative path; actual=" +
			reference)

	fastdds_profile_path = canonical_regular_file_within_root(
		os.path.join(os.path.dirname(profile_path), reference),
		config_root,
		profile_path + ":fastdds_profile")

	try:
		xml_root = ElementTree.parse(fastdds_profile_path).getroot()
	except (OSError, ElementTree.ParseError) as error:
		raise RuntimeError(
			fastdds_profile_path +
			": malformed FastDDS XML; expected=well-formed XML; actual=" +
			str(error)) from error

	if xml_root.tag.rsplit("}", 1)[-1] != "dds":
		raise RuntimeError(
			fastdds_profile_path +
			": unexpected FastDDS XML root; expected=dds; actual=" +
			xml_root.tag)

	return fastdds_profile_path
