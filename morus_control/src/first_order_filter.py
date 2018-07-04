class FirstOrderFilter:

    def __init__(self, b0, b1, a1):
        self.b0 = b0
        self.b1 = b1
        self.a1 = a1

        self.y_old = 0
        self.u_old = 0

    def compute(self, u_k):

        y_k = self.b0 * u_k + self.b1 * self.u_old + self.a1 * self.y_old
        self.y_old = y_k
        self.u_old = u_k

        return y_k