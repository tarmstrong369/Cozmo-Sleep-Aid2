import time


# AirborneTracker uses the IR sensor located on the bottom of the Cozmo to determine if Cozmo has been picked up or not.
# To prevent mis-reads and rapid flip-flopping, a small buffer has been incorporated.
class AirborneTracker:
    def __init__(self):
        self.is_airborne = False
        self.buffer = 0
        self.buffer_max = 4
        self.last_time = time.time()

    def report(self, picked_up):
        if picked_up:
            self.buffer = self.buffer + 1
            if self.buffer > self.buffer_max / 2.:
                self.buffer = self.buffer_max
                if not self.is_airborne:
                    self.is_airborne = True
                    self.last_time = time.time()
        else:
            self.buffer = self.buffer - 1
            if self.buffer < self.buffer_max / 2.:
                self.buffer = 0
                if self.is_airborne:
                    self.is_airborne = False
                    self.last_time = time.time()
