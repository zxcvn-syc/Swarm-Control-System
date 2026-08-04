import numpy as np
from logger_config import logger

class AuctionEngine:
    def __init__(self, agent_list, task_list):
        self.agents = agent_list
        self.tasks = task_list
        self.N = len(agent_list)
        self.M = len(task_list)
        self.utility_matrix = np.zeros((self.N, self.M))

    def generate_utility_matrix(self):
        """生成效用矩阵（展示用，数值越大越优）"""
        for i, ag in enumerate(self.agents):
            for j, task in enumerate(self.tasks):
                self.utility_matrix[i, j] = ag.compute_utility(task)
        logger.info(f"初始效用矩阵（数值越大越优）:\n{self.utility_matrix}")

    def bid_allocation(self):
        """
        逐任务效用最大化拍卖：
        每个任务选择效用最大的可行载体
        """
        allocation_result = {}

        for task in self.tasks:
            best_util = -float('inf')
            winner_idx = -1

            for idx, ag in enumerate(self.agents):
                util = ag.compute_utility(task)
                if util > best_util:
                    best_util = util
                    winner_idx = idx

            if winner_idx != -1:
                winner = self.agents[winner_idx]
                winner.assign_task(task)
                allocation_result[task.tid] = winner.aid
                logger.info(
                    f"任务{task.tid} → {winner.aid}，效用值：{round(best_util, 2)}，"
                    f"负载：{len(winner.task_list)}/{winner.max_load}，预计完成：{winner.current_time:.1f}"
                )
            else:
                logger.warning(f"任务{task.tid} 无可用载体，流拍")

        # 统计
        load_num = [len(ag.task_list) for ag in self.agents]
        logger.info(f"最终负载分布：{load_num}，平均：{np.mean(load_num):.2f}，方差：{np.var(load_num):.2f}")
        logger.info(f"分配结果：{allocation_result}")
        return allocation_result