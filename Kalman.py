from cProfile import label
import numpy as np
import matplotlib.pyplot as plt
from drawnow import *


def bringv(beaconinput):
    beacon_value = []
    for rssi in beaconinput:
        rssi = rssi.rstrip()
        a, b = map(float, rssi.split("\t"))
        beacon_value.append((b))
    return beacon_value


def makeplot(inputs, location):
    x = [i for i in range(len(inputs))]
    y = [v for v in inputs]
    plt.plot(x, y, label=location)


def SimpleKalman2(z):
    global x, P, A, H, Q, R

    xp = A.dot(x)
    Pp = A.dot(P).dot(A.T) + Q

    K = Pp.dot(H.T).dot(np.linalg.inv(H.dot(P).dot(H.T) + R))

    x = xp + K.dot(z - H.dot(xp))

    P = Pp - K.dot(H).dot(Pp)
    return x, P, K


left_input = list(open("beacon_left.txt"))
right_input = list(open("beacon_right.txt"))
front_input = list(open("beacon_front.txt"))
beacon_left = bringv(left_input)
beacon_right = bringv(right_input)
beacon_front = bringv(front_input)

z_left = []
x = beacon_left[0]
P = np.array([[6]])
A = np.array([[1]])
H = np.array([[1]])
Q = np.array([[0]])
R = np.array([[4]])

for i in range(1, len(beacon_left)):
    z = beacon_left[i]

    volt, Cov, Kg = SimpleKalman2(z)
    z_left.append(volt[0][0])


z_right = []
x = beacon_right[0]
P = np.array([[6]])
A = np.array([[1]])
H = np.array([[1]])
Q = np.array([[0]])
R = np.array([[4]])
for i in range(1, len(beacon_right)):
    z = beacon_right[i]

    volt, Cov, Kg = SimpleKalman2(z)
    z_right.append(volt[0][0])

z_front = []
x = beacon_front[0]
P = np.array([[6]])
A = np.array([[1]])
H = np.array([[1]])
Q = np.array([[0]])
R = np.array([[4]])

for i in range(1, len(beacon_front)):
    z = beacon_front[i]
    volt, Cov, Kg = SimpleKalman2(z)
    z_front.append(volt[0][0])

# makeplot(z_front, "Kalman front")
# makeplot(z_right, "Kalman right")
# makeplot(z_left, "Kalman left")

# plt.xlim(0, len(z_left))
# plt.ylim(-90, -60)
# plt.grid()
# plt.legend(loc=1)
# plt.show()


class AP:
    def __init__(self, x, y, distance):
        self.x = x
        self.y = y
        self.distance = distance


class Trilateration:
    def __init__(self, AP1, AP2, AP3):
        self.AP1 = AP1
        self.AP2 = AP2
        self.AP3 = AP3

    def calcUserLocation(self):
        A = 2 * (self.AP2.x - self.AP1.x)
        B = 2 * (self.AP2.y - self.AP1.y)
        C = (
            self.AP1.distance**2
            - self.AP2.distance**2
            - self.AP1.x**2
            + self.AP2.x**2
            - self.AP1.y**2
            + self.AP2.y**2
        )
        D = 2 * (self.AP3.x - self.AP2.x)
        E = 2 * (self.AP3.y - self.AP2.y)
        F = (
            self.AP2.distance**2
            - self.AP3.distance**2
            - self.AP2.x**2
            + self.AP3.x**2
            - self.AP2.y**2
            + self.AP3.y**2
        )

        user_x = ((F * B) - (E * C)) / ((B * D) - (E * A))
        user_y = ((F * A) - (D * C)) / ((A * E) - (D * B))
        return user_x, user_y


def distance(rssi):
    d = 10 ** ((-68 - rssi) / (10 * 2))
    return d


dis_left = []
for i in z_left:
    dis_left.append(distance(i))
dis_right = []
for i in z_right:
    dis_right.append(distance(i))

dis_front = []
for i in z_front:
    dis_front.append(distance(i))

# makeplot(dis_front, "dis front")
# makeplot(dis_right, "dis right")
# makeplot(dis_left, "dis left")


# plt.grid()
# plt.legend(loc=1)
# plt.show()

plt.plot(-2, -4, "o", label="ap1")
plt.plot(1.5, -4, "o", label="ap2")
plt.plot(-0.25, 3, "o", label="ap3")


if __name__ == "__main__":
    fig = plt.figure(1)

    def show_plot():
        global userx, usery
        plt.plot(-2, -4, "o", label="ap1")
        plt.plot(1.5, -4, "o", label="ap2")
        plt.plot(-0.25, 3, "o", label="ap3")
        plt.plot(userx, usery, "o", label="user")
        plt.legend()
        plt.xlabel("x축")
        plt.ylabel("y축")

    for i in range(20, len(z_front), 3):
        ap1 = AP(-2, 0, dis_left[i])
        ap2 = AP(2, 0, dis_right[i])
        ap3 = AP(0, 3, dis_front[i])
        tril = Trilateration(ap1, ap2, ap3)
        userx, usery = tril.calcUserLocation()
        drawnow(show_plot)
    plt.show()
