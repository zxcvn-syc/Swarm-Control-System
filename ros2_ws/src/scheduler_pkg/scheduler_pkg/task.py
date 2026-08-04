class Task:
    def __init__(self, tid: str, pos: list, reward: float, priority: int,
                 release_time: float, deadline: float, service_time: float = 10.0):
        self.tid = tid
        self.pos = pos
        self.reward = reward          # 任务收益（效用函数用）
        self.priority = priority      # 1~5，影响权重
        self.release_time = release_time
        self.deadline = deadline
        self.service_time = service_time

    def __repr__(self):
        return f"<{self.tid} at {self.pos}, reward={self.reward}, window=[{self.release_time}, {self.deadline}]>"