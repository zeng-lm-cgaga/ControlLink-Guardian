import os
from xml.etree import ElementTree

import yaml


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
