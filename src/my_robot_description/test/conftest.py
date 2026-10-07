"""pytest 配置：把脚本式的测试文件排除在自动发现之外。

背景
----
`test/` 下有两类测试，运行方式不同：

| 文件 | 类型 | 运行方式 |
|---|---|---|
| `test_protocol_and_kinematics.py` | 纯函数 **pytest** 测试 | `pytest test/` |
| `test_hil_end_to_end.py` | **脚本式** HIL 测试（需要 PTY 和真节点） | `python3 test_hil_end_to_end.py` |
| `test_bringup_smoke.py` | **脚本式** launch 冒烟测试 | `python3 test_bringup_smoke.py` |

后两个的函数签名是 `fn(failures, harness)` / `fn(failures)`，**不是 pytest fixture**。
pytest 会自动发现 `test_*.py` 里的 `test_*` 函数，于是把
`test_heartbeat_timeout_safe_state(f)` 里的 `f` 当成 fixture 名，报一堆
`fixture 'f' not found`。

两个候选修法：

1. 把脚本式文件改名成 `hil_end_to_end.py`（去掉 `test_` 前缀），pytest 就不收集了
   —— 但这会让文档里已经写好的路径、以及本文件里三处引用全部失效，还要改
   `acg720_protocol.md` / `real_robot_plan.md` 里的链接。
2. 用 `collect_ignore` 排除它们 —— 文件名保持不变，链接不受影响。

选 2。**注意排除的是"自动发现"，文件本身照旧可以 `python3` 直接跑**，
所以什么都没损失。
"""

#: 这两个文件由脚本方式运行，不是 pytest 用例（见模块 docstring）
collect_ignore = [
    "test_hil_end_to_end.py",
    "test_bringup_smoke.py",
]
