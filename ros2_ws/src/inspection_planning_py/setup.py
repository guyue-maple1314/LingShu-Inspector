from setuptools import find_packages, setup

package_name = "inspection_planning_py"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="青岛港湾职业技术学院 灵枢智行团队",
    maintainer_email="lszx_byc@qdgw.edu.cn",
    author="青岛港湾职业技术学院 灵枢智行团队",
    author_email="lszx_byc@qdgw.edu.cn",
    description="Python planning and decision layer for the quadruped inspection robot "
    "(c) 2026 qdgw / lszx_byc, all rights reserved.",
    license="proprietary",
    entry_points={
        "console_scripts": [
            "py_heartbeat_node = inspection_planning_py.connectivity.py_heartbeat_node:main",
            "llm_online_decision_node = inspection_planning_py.tech_1_1.llm_online_decision_node:main",
            "goal_replanner_node = inspection_planning_py.tech_1_1.goal_replanner_node:main",
            "candidate_task_pool_node = inspection_planning_py.tech_1_2.candidate_task_pool_node:main",
            "energy_constraint_node = inspection_planning_py.tech_1_2.energy_constraint_node:main",
            "preemption_scheduler_node = inspection_planning_py.tech_1_2.preemption_scheduler_node:main",
            "narrow_corridor_path_planner_node = inspection_planning_py.tech_1_4.narrow_corridor_path_planner_node:main",
            "grating_motion_supervisor_node = inspection_planning_py.tech_1_5.grating_motion_supervisor_node:main",
            "fusion_navigation_planner_node = inspection_planning_py.tech_1_6.fusion_navigation_planner_node:main",
            "semantic_annotation_node = inspection_planning_py.tech_1_7.semantic_annotation_node:main",
            "thermal_anomaly_decision_node = inspection_planning_py.tech_1_8.thermal_anomaly_decision_node:main",
            "adaptive_deep_filter_node = inspection_planning_py.tech_1_9.adaptive_deep_filter_node:main",
            "acoustic_fault_diagnosis_node = inspection_planning_py.tech_1_9.acoustic_fault_diagnosis_node:main",
        ],
    },
)
