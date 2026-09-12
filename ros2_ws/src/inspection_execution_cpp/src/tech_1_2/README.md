# tech_1_2 风险与能耗约束的动态任务抢占与恢复（C++ 执行侧）

- task_lifecycle_executor：Action 启动 / 取消 / 抢占 / 结束
- task_progress_store：保存与恢复任务进度
- task_resume_executor：从合法进度恢复
- preemption_latency_monitor：抢占响应耗时测量

