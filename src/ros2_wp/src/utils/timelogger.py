import time

class TimeLogger:
    def __init__(self, name: str):
        self.name = name
        self.times = {}
        self.start_time = None

    def start(self):
        self.times["start"] = time.time()

    def record(self, name: str):
        self.times[name] = time.time()

    def log(self):
        deltas = {}
        keys = list(self.times.keys())
        for i in range(1, len(keys)):
            deltas[keys[i]] = self.times[keys[i]] - self.times[keys[i - 1]]
        deltas["total"] = time.time() - self.times.get("start")
        return deltas

