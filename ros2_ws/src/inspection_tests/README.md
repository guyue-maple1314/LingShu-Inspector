# inspection_tests

按“接口测试 → 单元测试 → 集成测试 → 性能测试 → 验收测试”组织。

- unit_cpp / unit_python：与源码同编号的单元测试。
- integration：三层连通、任务抢占、传感器到诊断、告警到语义位置。
- performance：500 ms 切换、500 Hz 足力、1000 Hz IMU、30 fps 红外。
- acceptance：九项技术指标验收。
- fixtures：测试数据，不放算法源码。
