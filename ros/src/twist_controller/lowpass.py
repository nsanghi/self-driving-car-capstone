class LowPassFilter(object):

    def __init__(self, tau, ts):
        self.ts       = ts
        self.last_val = 0.0
        self.ready    = False
        self.set(tau)


    def set(self, tau):
        self.a = 1.0 / (tau / self.ts + 1.0)
        self.b = tau / self.ts / (tau / self.ts + 1.0);


    def filter(self, val):
        if self.ready:
            val = self.a * val + self.b * self.last_val
        else:
            self.ready = True
        self.last_val = val
        return val
