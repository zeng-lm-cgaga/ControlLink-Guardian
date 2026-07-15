#pragma once

#include <filesystem>
#include <stdexcept>
#include <string>
#include <string_view>

#include "control_link_contract/model.hpp"

namespace control_link_contract
{

class ContractError final : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

// 解析内存文本，source_name 仅用于生成可定位的结构化错误。
GatewayContractPtr parse_gateway_contract_text(
  std::string_view yaml_text, std::string source_name = "<memory>");

// 从唯一配置源加载文件；读取或解析失败时抛出 ContractError。
GatewayContractPtr load_gateway_contract(const std::filesystem::path & path);

}  // namespace control_link_contract
