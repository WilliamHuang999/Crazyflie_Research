import numpy as np


class Data:
    def __init__(self, length):
        self.array = np.empty((5, length))

    array = np.empty((5, 5))
    depth = 0
    reset_count = 0

    def addSeries(self, t, roll, pitch, yaw, thrust):
        series = np.array([t, roll, pitch, yaw, thrust])
        print(series)

        if self.depth >= self.array.shape[1]:
            series_c = np.array([[t], [roll], [pitch], [yaw], [thrust]])
            self.array = np.hstack((self.array, series_c))
        else:
            self.array[:, self.depth] = series

        self.depth += 1

    def save(self):
        np.savetxt("data.txt", self.array)

    def toString(self):
        return f"{myData.t}: {round(myData.roll, 2)}, {round(myData.pitch, 2)}, {round(myData.yaw, 2)}, {round(myData.thrust)}"


if __name__ == "__main__":
    myData = Data(3)

    myData.addSeries(1, 2, 3, 4, 5)
    myData.addSeries(6, 7, 8, 9, 10)

    myData.addSeries(11, 12, 13, 14, 15)
    myData.addSeries(16, 17, 18, 19, 20)
    myData.addSeries(21, 22, 23, 24, 25)

    myData.save()
