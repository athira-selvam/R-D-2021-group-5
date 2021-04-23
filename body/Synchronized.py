from abc import ABC
import time
from math import ceil


class Synchronized(ABC):

    @staticmethod
    def synchronize(sync_interval):
        if sync_interval == 0:
            return
        next_second = ceil(time.time()) * 1000
        next_tick = next_second + (next_second % sync_interval)
        print("sync at: " + str(next_tick) + ", sync_interval: " + str(sync_interval))
        wait_for = (next_tick - time.time() * 1000) / 1000
        if wait_for > 0:
            time.sleep(wait_for)
