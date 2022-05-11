class Drone():
    def __init__(self,m,n, time_0=None):
        """
        m, n 计算位置的点数
        time_0 基站与Drone连接的时间

        """

        self.m  = m
        self.n = n
        self.time_0 = time_0

    def get_time(self):
        return self.time_0
    def get_m(self):
        return self.m
    def get_n(self):
        return self.n

a = []
for i in range(3):
    time = 8 +i
    drone = Drone(m=i, n = i+1, time_0=time)
    a.append(drone)

print(a[1].m)

