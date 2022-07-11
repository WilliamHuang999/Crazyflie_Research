import numpy as np
import matplotlib.pyplot as plt


class Data:
    def __init__(self, length):
        self.array = np.empty((5, length))

    array = np.empty((5, 5))
    depth = 0

    def addSeries(self, t, roll, pitch, yaw, thrust):
        series = np.array([t, roll, pitch, yaw, thrust])

        if self.depth >= self.array.shape[1]:
            series_c = np.array([[t], [roll], [pitch], [yaw], [thrust]])
            self.array = np.hstack((self.array, series_c))
        else:
            self.array[:, self.depth] = series
        self.depth += 1

        self.array[1, :] = np.unwrap(self.array[1, :])
        self.array[2, :] = np.unwrap(self.array[2, :])
        self.array[3, :] = np.unwrap(self.array[3, :])

    def getSeries(self):
        return self.array[:, self.depth - 1]

    def plot(self):

        plt.plot(self.array[0, 0 : self.depth - 1], self.array[1, 0 : self.depth - 1], label="roll")
        plt.plot(self.array[0, 0 : self.depth - 1], self.array[2, 0 : self.depth - 1], label="pitch")
        plt.plot(self.array[0, 0 : self.depth - 1], self.array[3, 0 : self.depth - 1], label="yaw")
        plt.plot(self.array[0, 0 : self.depth - 1], self.array[4, 0 : self.depth - 1], label="thrust")
        # plt.plot(self.array[0, :], self.array[4, :], label="thrust")
        plt.legend()
        plt.show()

    def save(self, path):
        np.savetxt(f"{path}/data.txt", self.array)


if __name__ == "__main__":
    myData = Data(2000)

    myData.addSeries(1, 2, 3, 4, 5)
    myData.addSeries(6, 7, 8, 9, 10)

    myData.addSeries(11, 12, 13, 14, 15)
    myData.addSeries(16, 17, 18, 19, 20)
    myData.addSeries(21, 22, 23, 24, 25)

    phase = np.linspace(0.0, 20.0, 1000) % 2 * np.pi

    for i in range(phase.size):
        myData.addSeries(i, phase[i], 0, 0, 0)

    myData.plot()
