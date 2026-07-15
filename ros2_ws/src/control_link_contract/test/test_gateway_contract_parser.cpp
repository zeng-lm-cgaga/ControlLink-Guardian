#include <fstream>
#include <sstream>
#include <string>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "control_link_contract/parser.hpp"

namespace control_link_contract
{
namespace
{

std::string read_valid_contract()
{
  std::ifstream input(CONTROL_LINK_TEST_CONTRACT_PATH);
  if (!input) {
    throw std::runtime_error("无法读取测试 Contract: " CONTROL_LINK_TEST_CONTRACT_PATH);
  }
  std::ostringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

std::string parse_error(const YAML::Node & node)
{
  try {
    static_cast<void>(parse_gateway_contract_text(YAML::Dump(node), "test.yaml"));
  } catch (const ContractError & error) {
    return error.what();
  }
  ADD_FAILURE() << "预期 ContractError，但解析成功";
  return {};
}

std::string parse_text_error(const std::string & text)
{
  try {
    static_cast<void>(parse_gateway_contract_text(text, "test.yaml"));
  } catch (const ContractError & error) {
    return error.what();
  }
  ADD_FAILURE() << "预期 ContractError，但解析成功";
  return {};
}

void expect_error(
  const YAML::Node & root, const std::string & expected_path,
  const std::string & expected_rule)
{
  const std::string error = parse_error(root);
  EXPECT_NE(error.find("test.yaml:" + expected_path), std::string::npos) << error;
  EXPECT_NE(error.find(expected_rule), std::string::npos) << error;
  EXPECT_NE(error.find("expected="), std::string::npos) << error;
  EXPECT_NE(error.find("actual="), std::string::npos) << error;
  EXPECT_NE(error.find("hint="), std::string::npos) << error;
}

TEST(GatewayContractParser, LoadsRepositoryContract)
{
  const auto contract = load_gateway_contract(CONTROL_LINK_TEST_CONTRACT_PATH);

  ASSERT_NE(contract, nullptr);
  EXPECT_EQ(contract->schema_version, 1U);
  EXPECT_EQ(contract->contract_id, "control_link_gateway_v1");
  EXPECT_DOUBLE_EQ(contract->gateway.output_rate_hz, 50.0);
  EXPECT_EQ(contract->gateway.command_timeout_ms, 100U);
  EXPECT_DOUBLE_EQ(contract->limits.max_abs_linear_velocity_mps, 1.0);
  ASSERT_EQ(contract->qos_profiles.count("control_input"), 1U);
  ASSERT_TRUE(contract->qos_profiles.at("control_input").deadline_ms.has_value());
  EXPECT_EQ(*contract->qos_profiles.at("control_input").deadline_ms, 80U);
  ASSERT_EQ(contract->qos_profiles.count("canonical_output"), 1U);
  ASSERT_TRUE(contract->qos_profiles.at("canonical_output").deadline_ms.has_value());
  EXPECT_EQ(*contract->qos_profiles.at("canonical_output").deadline_ms, 40U);
  EXPECT_EQ(contract->critical_endpoints.size(), 2U);
}

TEST(GatewayContractParser, RejectsMissingRequiredFieldWithPath)
{
  YAML::Node root = YAML::Load(read_valid_contract());
  root.remove("schema_version");

  const std::string error = parse_error(root);
  EXPECT_NE(error.find("test.yaml:schema_version"), std::string::npos);
  EXPECT_NE(error.find("expected="), std::string::npos);
}

TEST(GatewayContractParser, RejectsUnknownNestedFieldWithPath)
{
  YAML::Node root = YAML::Load(read_valid_contract());
  root["gateway"]["silent_default"] = 123;

  const std::string error = parse_error(root);
  EXPECT_NE(error.find("test.yaml:gateway.silent_default"), std::string::npos);
  EXPECT_NE(error.find("unknown field"), std::string::npos);
}

TEST(GatewayContractParser, RejectsNonPositiveOutputRateWithPath)
{
  YAML::Node root = YAML::Load(read_valid_contract());
  root["gateway"]["output_rate_hz"] = 0;

  const std::string error = parse_error(root);
  EXPECT_NE(error.find("test.yaml:gateway.output_rate_hz"), std::string::npos);
  EXPECT_NE(error.find("expected="), std::string::npos);
}

TEST(GatewayContractParser, RejectsUnknownQosEnumWithPath)
{
  YAML::Node root = YAML::Load(read_valid_contract());
  root["qos_profiles"]["control_input"]["reliability"] = "sometimes";

  const std::string error = parse_error(root);
  EXPECT_NE(
    error.find("test.yaml:qos_profiles.control_input.reliability"), std::string::npos);
  EXPECT_NE(error.find("expected="), std::string::npos);
}

TEST(GatewayContractParser, RejectsMissingQosReferenceWithPath)
{
  YAML::Node root = YAML::Load(read_valid_contract());
  root["output"]["qos_profile"] = "missing_profile";

  const std::string error = parse_error(root);
  EXPECT_NE(error.find("test.yaml:output.qos_profile"), std::string::npos);
  EXPECT_NE(error.find("missing_profile"), std::string::npos);
}

TEST(GatewayContractParser, RejectsMalformedYamlWithStructuredError)
{
  const std::string error = parse_text_error("gateway: [unterminated");

  EXPECT_NE(error.find("test.yaml:root"), std::string::npos);
  EXPECT_NE(error.find("invalid YAML"), std::string::npos);
  EXPECT_NE(error.find("hint="), std::string::npos);
}

TEST(GatewayContractParser, RejectsDuplicateField)
{
  const std::string valid_contract = read_valid_contract();
  const std::string duplicate = "schema_version: 1\n" + valid_contract;

  const std::string error = parse_text_error(duplicate);
  EXPECT_NE(error.find("test.yaml:schema_version"), std::string::npos);
  EXPECT_NE(error.find("duplicate field"), std::string::npos);
}

TEST(GatewayContractParser, RejectsTimeoutShorterThanOutputPeriod)
{
  YAML::Node root = YAML::Load(read_valid_contract());
  root["gateway"]["command_timeout_ms"] = 10;

  expect_error(root, "gateway.command_timeout_ms", "timeout shorter than output period");
}

TEST(GatewayContractParser, RejectsInvalidGatewayNodeFqn)
{
  YAML::Node root = YAML::Load(read_valid_contract());
  root["gateway"]["node_fqn"] = "control_link/gateway";

  expect_error(root, "gateway.node_fqn", "invalid ROS node FQN");
}

TEST(GatewayContractParser, RejectsNonPositiveVelocityLimit)
{
  YAML::Node root = YAML::Load(read_valid_contract());
  root["limits"]["max_abs_linear_velocity_mps"] = -0.1;

  expect_error(root, "limits.max_abs_linear_velocity_mps", "non-positive limit");
}

TEST(GatewayContractParser, RejectsFutureSkewLargerThanCommandTimeout)
{
  YAML::Node root = YAML::Load(read_valid_contract());
  root["limits"]["max_future_skew_ms"] = 101;

  expect_error(root, "limits.max_future_skew_ms", "future skew exceeds command timeout");
}

TEST(GatewayContractParser, RejectsZeroKeepLastDepth)
{
  YAML::Node root = YAML::Load(read_valid_contract());
  root["qos_profiles"]["control_input"]["depth"] = 0;

  expect_error(root, "qos_profiles.control_input.depth", "invalid keep_last depth");
}

TEST(GatewayContractParser, RejectsIncompleteLivelinessPolicy)
{
  YAML::Node root = YAML::Load(read_valid_contract());
  root["qos_profiles"]["control_input"].remove("liveliness_lease_duration_ms");

  expect_error(root, "qos_profiles.control_input", "incomplete liveliness policy");
}

TEST(GatewayContractParser, RejectsDuplicateCriticalEndpointId)
{
  YAML::Node root = YAML::Load(read_valid_contract());
  root["critical_endpoints"][1]["id"] = root["critical_endpoints"][0]["id"];

  expect_error(root, "critical_endpoints[1].id", "duplicate endpoint id");
}

TEST(GatewayContractParser, RejectsCriticalEndpointOutsidePublicContract)
{
  YAML::Node root = YAML::Load(read_valid_contract());
  root["critical_endpoints"][0]["topic"] = "/private/control";

  expect_error(root, "critical_endpoints[0].topic", "endpoint is outside public Contract");
}

TEST(GatewayContractParser, RejectsEndpointMaxCountBelowMinCount)
{
  YAML::Node root = YAML::Load(read_valid_contract());
  root["critical_endpoints"][0]["min_count"] = 2;
  root["critical_endpoints"][0]["max_count"] = 1;

  expect_error(root, "critical_endpoints[0].max_count", "max_count smaller than min_count");
}

TEST(GatewayContractParser, RejectsUnknownRuntimeAction)
{
  YAML::Node root = YAML::Load(read_valid_contract());
  root["critical_endpoints"][0]["runtime_loss_action"] = "ignore";

  expect_error(root, "critical_endpoints[0].runtime_loss_action", "unknown runtime action");
}

}  // namespace
}  // namespace control_link_contract
