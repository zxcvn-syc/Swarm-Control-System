from agent import Agent
from task import Task
from logger_config import logger

class GreedyScheduler:
    def __init__(self, agent_list: list[Agent], task_list: list[Task], sim_time: int = 0):
        self.agents = agent_list
        self.all_tasks = task_list
        self.sim_time = sim_time
        self.final_allocation = {}  # 存储分配结果 {task_id: agent_id}

    def run_greedy_allocate(self):
        """执行贪心调度主逻辑"""
        # 1. 过滤未过期有效任务
        valid_tasks = [t for t in self.all_tasks if not t.is_expired(self.sim_time)]
        logger.info(f"当前仿真时间{self.sim_time}，有效任务数量：{len(valid_tasks)}")
        if not valid_tasks:
            logger.warning("无有效任务，调度结束")
            return self.final_allocation

        # 2. 遍历每一个有效任务，分配最优载体
        for task in valid_tasks:
            best_agent = None
            min_cost = float("inf")
            # 遍历所有载体，筛选满足约束的设备
            for agent in self.agents:
                if agent.can_accept_task(task.pos):
                    current_cost = agent.get_total_cost(task.pos)
                    # 更新最小代价载体
                    if current_cost < min_cost:
                        min_cost = current_cost
                        best_agent = agent

            # 3. 分配判定
            if best_agent is not None:
                # 绑定任务与载体
                task.assign_agent = best_agent.aid
                best_agent.task_list.append(task)
                self.final_allocation[task.tid] = best_agent.aid
                logger.info(f"任务{task.tid} 分配至载体{best_agent.aid}，综合代价：{min_cost:.2f}")
            else:
                logger.warning(f"任务{task.tid}无可用载体，分配失败")

        return self.final_allocation