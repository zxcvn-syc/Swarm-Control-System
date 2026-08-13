try:
    from .agent import Agent
    from .task import Task
    from .logger_config import logger
except ImportError:  # pragma: no cover - supports direct script execution
    from agent import Agent
    from task import Task
    from logger_config import logger

# ---------- 任务配置 ----------
task_info = [
    ("T01", [2,3], 50, 5, 0,   60, 10),
    ("T02", [5,1], 45, 4, 5,   80, 8),
    ("T03", [1,7], 40, 3, 10,  50, 12),
    ("T04", [8,4], 60, 5, 15, 100, 15),
    ("T05", [4,6], 35, 2, 20,  70, 10),
    ("T06", [7,2], 55, 4, 25,  90, 10),
]
task_list = [Task(*t) for t in task_info]

# ---------- 载体配置 ----------
agent_list = [
    Agent("UAV0", "UAV", [0,5], 100, 3, 1.0, 2.0),
    Agent("UAV1", "UAV", [10,5], 100, 3, 1.0, 2.0),
    Agent("UGV0", "UGV", [20,5], 80, 3, 0.6, 1.0),
    Agent("UGV1", "UGV", [30,5], 80, 3, 0.6, 1.0),
    Agent("UGV2", "UGV", [40,5], 80, 3, 0.6, 1.0),
]

logger.info("===== 异构集群（2 UAV + 3 UGV）动态效用拍卖启动 =====")

# ---------- 测试 ----------
uav0 = agent_list[0]
t0 = task_list[0]
logger.info(f"{uav0.aid} 空载对 {t0.tid} 效用：{uav0.compute_utility(t0):.2f}")

# 模拟负载影响
uav0.task_list.append(t0)
logger.info(f"{uav0.aid} 负载1对 {t0.tid} 效用：{uav0.compute_utility(t0):.2f}")
uav0.task_list.clear()
uav0.battery = 100.0
uav0.current_time = 0.0
logger.info("测试状态已恢复")

# ---------- 运行拍卖 ----------
from auction_engine import AuctionEngine
engine = AuctionEngine(agent_list, task_list)
engine.generate_utility_matrix()
engine.bid_allocation()
logger.info("===== 第3周调度完成 =====")