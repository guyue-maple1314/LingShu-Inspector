"""工程一致性检查（静态扫描，无需 ROS）。

这些用例专门盯“节点接线层”的错误——它们曾经被纯逻辑单测漏掉：
- C++ / Python 两套 Topic 命名表必须一致；
- topics.yaml 必须覆盖全部命名表条目；
- 接口包 CMakeLists 登记的消息文件必须与磁盘一致；
- 节点里不允许硬编码 Topic 字面量（必须用命名表常量）；
- 命名表里不允许存在“谁都没用”的死 Topic；
- launch 里用到的可执行文件必须真实存在（CMake / setup.py 已注册）；
- 数据参数清单里的消息/话题数量必须与实际一致；
- 每项技术的验收目录必须有 README 与 scenarios。
"""

import re
import unittest
from pathlib import Path


#: 命名表里声明、但按设计由上层适配器直接读取的原始传感器话题
#: （当前实现走 ImuAdapter/FootForceAdapter 等抽象接口，未经 ROS 话题；
#:   真实传感器接入后应改为订阅，并把名字从本清单移除）
RAW_SENSOR_TOPICS_PENDING = {
    "/imu",
    "/foot_force",
    "/lidar_scan",
    "/image",
    "/thermal_image",
    "/audio_multi_channel",
}

#: 由外部包提供、不属于本工程注册范围的可执行文件
EXTERNAL_LAUNCH_EXECUTABLES = {"rosbridge_websocket"}


def _root() -> Path:
    # <root>/ros2_ws/src/inspection_tests/unit_python/tech_1_1_to_1_9/test_*.py
    return Path(__file__).resolve().parents[5]


ROOT = _root()
SRC = ROOT / "ros2_ws" / "src"
PY_TOPICS = SRC / "inspection_planning_py" / "inspection_planning_py" / "common" / "topic_names.py"
CPP_TOPICS = (SRC / "inspection_execution_cpp" / "include" / "inspection_execution_cpp"
              / "common" / "topic_names.hpp")
TOPICS_YAML = SRC / "inspection_bringup" / "config" / "topics.yaml"
IFACE_CMAKE = SRC / "inspection_interfaces" / "CMakeLists.txt"
PARAM_TABLE = ROOT / "docs" / "数据参数清单.txt"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _py_topic_paths() -> dict:
    text = _read(PY_TOPICS)
    return dict(re.findall(r'^([A-Z_0-9]+)\s*=\s*"(/[^"]+)"', text, re.MULTILINE))


def _cpp_topic_paths() -> dict:
    text = _read(CPP_TOPICS)
    return dict(re.findall(r'inline constexpr const char\*\s+(k\w+)\s*=\s*"(/[^"]+)"',
                           text))


def _code_files(*suffixes: str):
    for path in SRC.rglob("*"):
        if path.is_file() and path.suffix in suffixes:
            yield path


class TestTopicNamingTables(unittest.TestCase):
    def test_python_and_cpp_paths_match(self):
        py_paths = set(_py_topic_paths().values())
        cpp_paths = set(_cpp_topic_paths().values())
        self.assertEqual(py_paths, cpp_paths,
                         f"仅 Python 有：{py_paths - cpp_paths}；"
                         f"仅 C++ 有：{cpp_paths - py_paths}")

    def test_topics_yaml_covers_naming_table(self):
        text = _read(TOPICS_YAML)
        yaml_paths = set(re.findall(r"name:\s*(/\S+?)[,\s}]", text))
        # topic yaml 只登记业务话题；原始传感器话题尚未经 ROS 话题接入，
        # 由适配器直读（见 RAW_SENSOR_TOPICS_PENDING 说明）。
        declared = set(_py_topic_paths().values()) - RAW_SENSOR_TOPICS_PENDING
        self.assertEqual(declared, yaml_paths,
                         f"topics.yaml 缺失：{declared - yaml_paths}；"
                         f"多余的：{yaml_paths - declared}")

    def test_no_dead_topic_in_naming_table(self):
        """命名表里每一条都必须有真实使用点（死 Topic 是接线错误的信号）。"""
        py_names = _py_topic_paths()
        cpp_names = {v: k for k, v in _cpp_topic_paths().items()}
        dead = []
        for path in sorted(set(py_names.values())):
            py_name = next((k for k, v in py_names.items() if v == path), None)
            cpp_name = cpp_names.get(path)
            used = 0
            for f in _code_files(".py"):
                if f == PY_TOPICS:
                    continue
                text = _read(f)
                if py_name and re.search(rf"\b{py_name}\b", text):
                    used += 1
            for f in _code_files(".cpp", ".hpp"):
                if f == CPP_TOPICS:
                    continue
                text = _read(f)
                if cpp_name and re.search(rf"\b{cpp_name}\b", text):
                    used += 1
            if used == 0 and path not in RAW_SENSOR_TOPICS_PENDING:
                dead.append(path)
        self.assertEqual(dead, [], f"命名表里存在无人使用的 Topic：{dead}")

    def test_no_hardcoded_topic_literal_in_nodes(self):
        """节点创建 publisher/subscription/client 时不允许写 Topic 字面量。"""
        pattern = re.compile(
            r"create_(?:publisher|subscription|client)\s*(?:<[^>]*>)?\s*\(|ActionClient\s*\(")
        offenders = []
        for f in _code_files(".py", ".cpp", ".hpp"):
            if f in (PY_TOPICS, CPP_TOPICS):
                continue
            text = _read(f)
            for match in pattern.finditer(text):
                window = text[match.start(): match.start() + 260]
                if re.search(r'"/[a-z_0-9]+"', window):
                    line = text[: match.start()].count("\n") + 1
                    offenders.append(f"{f.name}:{line}")
        self.assertEqual(offenders, [],
                         f"存在硬编码 Topic 字面量：{offenders}")


class TestInterfaceRegistration(unittest.TestCase):
    def test_msg_files_match_cmakelists(self):
        listed = set(re.findall(r'"(msg/[^"]+\.msg)"', _read(IFACE_CMAKE)))
        on_disk = {f"msg/{p.name}" for p in (SRC / "inspection_interfaces"
                                             / "msg").glob("*.msg")}
        self.assertEqual(listed, on_disk,
                         f"未登记：{on_disk - listed}；登记但不存在：{listed - on_disk}")

    def test_srv_and_action_files_match_cmakelists(self):
        cmake = _read(IFACE_CMAKE)
        for kind in ("srv", "action"):
            listed = set(re.findall(rf'"({kind}/[^"]+)"', cmake))
            on_disk = {f"{kind}/{p.name}"
                       for p in (SRC / "inspection_interfaces" / kind).glob(f"*.{kind}")}
            self.assertEqual(listed, on_disk, f"{kind} 登记不一致")


class TestLaunchWiring(unittest.TestCase):
    def test_launch_executables_are_registered(self):
        cmake_text = _read(SRC / "inspection_execution_cpp" / "CMakeLists.txt")
        cpp_targets = set(re.findall(r"add_executable\((\w+)", cmake_text))
        setup_text = _read(SRC / "inspection_planning_py" / "setup.py")
        py_targets = set(re.findall(r'"(\w+)\s*=', setup_text))
        registered = cpp_targets | py_targets

        missing = []
        for launch in (SRC / "inspection_bringup" / "launch").glob("*.launch.py"):
            for exe in re.findall(r'executable="([a-zA-Z0-9_]+)"', _read(launch)):
                if exe in EXTERNAL_LAUNCH_EXECUTABLES:
                    continue
                if exe not in registered:
                    missing.append(f"{launch.name} → {exe}")
        self.assertEqual(missing, [], f"launch 引用了未注册的可执行文件：{missing}")

    def test_each_tech_has_launch(self):
        launch_dir = SRC / "inspection_bringup" / "launch"
        missing = [f"tech_1_{i}.launch.py" for i in range(1, 10)
                   if not (launch_dir / f"tech_1_{i}.launch.py").exists()]
        self.assertEqual(missing, [], f"缺少按技术启动文件：{missing}")


class TestAcceptanceAndParameterTable(unittest.TestCase):
    def test_acceptance_dirs_have_docs(self):
        base = SRC / "inspection_tests" / "acceptance"
        missing = []
        for i in range(1, 10):
            d = base / f"tech_1_{i}"
            for name in ("README.md", "scenarios.md"):
                if not (d / name).exists():
                    missing.append(f"tech_1_{i}/{name}")
        self.assertEqual(missing, [], f"验收目录缺文档：{missing}")

    def test_parameter_table_counts_match_repo(self):
        text = _read(PARAM_TABLE)
        msg_count = int(re.search(r"【三】(\d+)\s*个消息", text).group(1))
        topic_count = int(re.search(r"Topic（(\d+)）", text).group(1))
        real_msgs = len(list((SRC / "inspection_interfaces" / "msg").glob("*.msg")))
        # Topic 计数只统计业务话题：排除 service/action 常量与尚未经 ROS 话题
        # 接入的原始传感器话题（后者在清单【一】按设备列出）。
        real_topics = len({
            path for name, path in _py_topic_paths().items()
            if not name.endswith(("_SERVICE", "_ACTION"))
        } - RAW_SENSOR_TOPICS_PENDING)
        self.assertEqual(msg_count, real_msgs, "数据参数清单的消息数量与实际不符")
        self.assertEqual(topic_count, real_topics, "数据参数清单的话题数量与实际不符")


class TestNoDeadPythonModule(unittest.TestCase):
    def test_every_module_is_referenced(self):
        """Python 侧不允许存在“写了但没人用”的模块（公共基础设施必须真正接入）。"""
        base = SRC / "inspection_planning_py" / "inspection_planning_py"
        modules = [p for p in base.rglob("*.py") if p.name != "__init__.py"]
        all_sources = list(SRC.rglob("*.py"))
        dead = []
        for module in modules:
            name = module.stem
            if not any(
                other != module and re.search(rf"\b{re.escape(name)}\b", _read(other))
                for other in all_sources
            ):
                dead.append(str(module.relative_to(base)))
        self.assertEqual(dead, [], f"未被任何文件引用的 Python 模块：{dead}")


if __name__ == "__main__":
    unittest.main()
