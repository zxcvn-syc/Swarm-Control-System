import math

class Agent:
    def __init__(self, aid: str, category: str, pos: list, battery: float,
                 max_load: int, unit_cost: float, speed: float,
                 service_time_per_task: float = 10.0):
        self.aid = aid
        self.category = category          # "UAV" 或 "UGV"
        self.pos = pos
        self.battery = battery
        self.max_load = max_load
        self.unit_cost = unit_cost
        self.speed = speed
        self.service_time_per_task = service_time_per_task

        self.task_list = []
        self.current_time = 0.0
        self.current_pos = pos[:]

        # ===== 效用函数权重基值（可调） =====
        self.base_alpha = 1.0      # 收益权重基值
        self.base_gamma = 0.3      # 能耗权重基值
        self.base_delta = 0.2      # 风险权重基值
        self.beta = 0.2            # 覆盖贡献权重（固定）

    # ---------- 基础方法 ----------
    def get_distance(self, target_pos):
        return math.hypot(self.pos[0] - target_pos[0], self.pos[1] - target_pos[1])

    def compute_travel_time(self, target_pos):
        dist = self.get_distance(target_pos)
        return dist / self.speed if self.speed > 0 else float('inf')

    def calc_total_energy(self, target_pos):
        """往返总能耗（UAV/UGV差异化）"""
        dist = self.get_distance(target_pos)
        if self.category == "UAV":
            return dist * self.unit_cost * 2.0
        else:
            return dist * self.unit_cost * 1.5

    # ---------- 硬约束检查 ----------
    def can_accept_task(self, task):
        """载重 + 电量 + 时间窗 三重约束"""
        if len(self.task_list) >= self.max_load:
            return False
        if self.calc_total_energy(task.pos) > self.battery:
            return False

        travel_time = self.compute_travel_time(task.pos)
        arrival = self.current_time + travel_time
        start = max(arrival, task.release_time)
        finish = start + task.service_time
        if finish > task.deadline:
            return False
        return True

    # ---------- 分配任务 ----------
    def assign_task(self, task):
        travel_time = self.compute_travel_time(task.pos)
        arrival = self.current_time + travel_time
        start = max(arrival, task.release_time)
        finish = start + task.service_time
        self.current_time = finish
        self.current_pos = task.pos[:]
        self.task_list.append(task)
        self.battery -= self.calc_total_energy(task.pos) * 0.5

    # ========== 动态权重计算 ==========
    def compute_dynamic_weights(self, task_priority, target_pos):
        """
        根据当前状态自适应调整 α、γ、δ
        - α（收益权重）：随任务优先级升高
        - γ（能耗权重）：随电量降低而增大
        - δ（风险权重）：随距离和负载增大
        """
        soc = max(0.1, self.battery / 100.0)          # 剩余电量比例
        threat = task_priority / 5.0                  # 威胁度 0.2~1.0
        dist = self.get_distance(target_pos)
        dist_factor = min(1.0, dist / 30.0)           # 距离因子 0~1
        load_ratio = len(self.task_list) / self.max_load if self.max_load > 0 else 0

        # 动态调整
        alpha = self.base_alpha * (1 + 0.8 * threat)                         # 高威胁 → 收益↑
        gamma = self.base_gamma * (1 + 1.5 * (1 - soc) + 0.5 * load_ratio)  # 低电量/高负载 → 能耗惩罚↑
        delta = self.base_delta * (1 + 0.6 * dist_factor + 0.3 * load_ratio)# 远距离/高负载 → 风险惩罚↑

        return alpha, gamma, delta

    # ========== 动态效用函数 ==========
    def compute_utility(self, task):
        """
        完整动态效用函数：
        U = α × R + β × C - γ × E - δ × D

        其中：
        R = 收益 × 距离折扣（越近收益越高，避免舍近求远）
        C = 覆盖贡献（预留，固定为1）
        E = 往返能耗
        D = 距离风险 + 负载风险
        """
        if not self.can_accept_task(task):
            return -float('inf')

        # ----- 1. 收益项 R -----
        dist = self.get_distance(task.pos)
        dist_discount = max(0.3, 1 - dist / 25.0)     # 最远折扣0.3
        R = task.reward * dist_discount

        # ----- 2. 覆盖贡献 C -----
        C = 1.0

        # ----- 3. 能耗代价 E -----
        E = self.calc_total_energy(task.pos)

        # ----- 4. 风险惩罚 D -----
        load_ratio = len(self.task_list) / self.max_load if self.max_load > 0 else 0
        D = (dist / 10.0) + (load_ratio * 3.0)

        # ----- 动态权重 -----
        alpha, gamma, delta = self.compute_dynamic_weights(task.priority, task.pos)

        # ----- 效用计算 -----
        utility = alpha * R + self.beta * C - gamma * E - delta * D
        return utility

    # ---------- 兼容旧接口 ----------
    def generate_bid(self, task_pos, task_priority):
        """保留旧报价函数（仅用于日志展示，实际竞标改用效用）"""
        dist = self.get_distance(task_pos)