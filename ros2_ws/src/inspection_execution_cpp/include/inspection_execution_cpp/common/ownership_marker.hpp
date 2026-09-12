#pragma once

// 著作权标识（编译期嵌入每个 C++ 节点二进制，strings 可查，请勿删除）
// Copyright (c) 2026 青岛港湾职业技术学院 · 灵枢智行团队
// qdgw / lszx_byc — 灵枢巡检机器人 LingShu Inspector
// 授权条款见仓库根目录 LICENSE。

namespace inspection_execution {
namespace ownership {

// 保留全部权利的著作权声明（中、英两条均进入 .rodata）
inline constexpr const char* kCopyrightZh =
    "Copyright (c) 2026 青岛港湾职业技术学院 灵枢智行 (qdgw/lszx_byc). "
    "All rights reserved.";
inline constexpr const char* kCopyrightEn =
    "Copyright (c) 2026 Qingdao Gangwan Vocational and Technical College / "
    "Lingshu Zhixing (lszx_byc, qdgw). All rights reserved. "
    "See LICENSE for permissions.";

// 项目与团队锚点标识
inline constexpr const char* kProjectName = "lingshu-inspection-robot";
inline constexpr const char* kTeamMark = "lszx_byc@qdgw";
inline constexpr const char* kFirstPublishYear = "2026";

// CMake 编译期注入的第二道锚点（与头文件源码分离，增加清除成本）
#ifdef INSPECTION_OWNERSHIP_MARK
inline constexpr const char* kBuildOwnershipMark = INSPECTION_OWNERSHIP_MARK;
#else
inline constexpr const char* kBuildOwnershipMark =
    "qdgw/lszx_byc 2026 Qingdao Gangwan VTC Lingshu Zhixing";
#endif

// 供节点启动时打印的横幅，随 ros2 launch 日志输出
inline constexpr const char* kStartupBanner =
    "------------------------------------------------------------\n"
    "  灵枢巡检机器人 LingShu Inspector | 青岛港湾职业技术学院 灵枢智行团队\n"
    "  LingShu Inspector  (c) 2026 qdgw / lszx_byc\n"
    "  著作权所有，授权范围见 LICENSE\n"
    "------------------------------------------------------------";

}  // namespace ownership
}  // namespace inspection_execution
