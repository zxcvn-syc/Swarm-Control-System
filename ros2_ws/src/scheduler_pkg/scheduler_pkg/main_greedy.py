# main_greedy.py 贪心基准对照组入口
from agent import Agent
from task import Task
from scheduler import GreedyScheduler
from logger_config import logger

if __name__ == "__main__":
    # 和拍卖用完全相同的载体、任务参数，保证实验公平对比
    agent_info = [
        ["UAV0", "UAV", [0, 0], 100, 2, 1.2],
        ["UAV1", "UAV", [1, 1], 95, 2, 1.1],
        ["UAV2", "UAV", [9, 0], 88, 2, 1.3],
        ["UGV0", "UGV", [2, 8], 120, 3, 0.6],
        ["UGV1", "UGV", [7, 7], 110, 3, 0.5],
    ]
    agent_list = []
    for aid, category, pos, battery, max_load, unit_cost in agent_info:
        agent_list.append(Agent(aid, category, pos, battery, max_load, unit_cost))
    logger.info("===== 5台载体、6项任务实例化完成 =====")

    task_info = [
        ["T01", [2, 3], 20, 5, 120],
        ["T02", [5, 1], 18, 4, 100],
        ["T03", [1, 7], 15, 3, 90],
        ["T04", [8, 4], 22, 5, 150],
        ["T05", [4, 6], 16, 2, 80],
        ["T06", [7, 2], 19, 4, 110],
    ]
    task_list = []
    for tid, pos, reward, priority, valid_time in task_info:
        task_list.append(Task(tid, pos, reward, priority, valid_time))

    # 运行贪心调度
    sim_time = 10
    scheduler = GreedyScheduler(agent_list, task_list, sim_time)
    greedy_res = scheduler.run_greedy_allocate()
    logger.info(f"贪心调度分配结果：{greedy_res}")
    logger.info("===== 贪心基准调度流程执行完毕 =====")