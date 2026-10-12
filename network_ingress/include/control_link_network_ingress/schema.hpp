#pragma once

#include <string>

namespace control_link::network {

// 原始 .proto 字节的 SHA256（32B），由唯一生成/编译 target 提供，不包含 gRPC stub 布局。
std::string schema_fingerprint();

}
