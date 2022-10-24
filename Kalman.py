import numpy as np
import matplotlib.pyplot as plt


class KalmanFilter:
    def __init__(self, processNoise=0.005, measurementNoise=20):
        super(KalmanFilter, self).__init__()
        self.isInitialized = False
        self.processNoise = processNoise
        self.measurementNoise = measurementNoise
        self.predictedRSSI = 0
        self.errorCovariance = 0

    def filtering(self, rssi):
        if not self.isInitialized:
            self.isInitialized = True
            priorRSSI = rssi
            priorErrorCovariance = 1
        else:
            priorRSSI = self.predictedRSSI
            priorErrorCovariance = self.errorCovariance + self.processNoise

        kalmanGain = priorErrorCovariance / (
            priorErrorCovariance + self.measurementNoise
        )
        self.predictedRSSI = priorRSSI + (kalmanGain * (rssi - priorRSSI))
        self.errorCovarianceRSSI = (1 - kalmanGain) * priorErrorCovariance

        return self.predictedRSSI


A = np.array([[6]])

print(A)


def bringv(beaconinput):
    beacon_value = []
    for rssi in beaconinput:
        rssi = rssi.rstrip()
        a, b = map(float, rssi.split("\t"))
        beacon_value.append((b))
    return beacon_value


left_input = list(open("beacon_left.txt"))
right_input = list(open("beacon_right.txt"))
front_input = list(open("beacon_front.txt"))
beacon_left = bringv(left_input)
beacon_right = bringv(right_input)
beacon_front = bringv(front_input)


def makeplot(inputs, location):
    x = [i for i in range(len(inputs))]
    y = [v for v in inputs]
    plt.plot(x, y, label=location)


# fig_left = makeplot(beacon_left, "beacon left")
# fig_right = makeplot(beacon_right, "beacon right")
# fig_front = makeplot(beacon_front, "beacon front")
# plt.xlabel("time"), plt.ylabel("RSSI")
# plt.grid()
# plt.legend(loc=1)
# plt.show()


prior = KalmanFilter(0.005, 20)

k_left = []
for i in range(len(beacon_left)):
    krssi = prior.filtering(beacon_left[i])
    k_left.append(krssi)
makeplot(k_left, "Kalman left")


# plt.grid()
# plt.legend(loc=1)
# plt.show()
