
import numpy as np

class GapFinder:

    def __init__(self, invalid_band_size, ceiling_m = 2,  running_average_length = 10):
        self.ceiling_m = ceiling_m
        self.invalid_band_size = invalid_band_size
        self.running_average = RunningAverage(size = running_average_length)
        self.depth_frame_units = -1
    
    def addFrame(self, depth_frame):
        self.depth_frame_units = depth_frame.get_units()
        depth_image = np.asanyarray(depth_frame.get_data())
        (IMG_HEIGHT, IMG_WIDTH) = np.shape(depth_image)

        # Cut off invalid depth band (and equal width on opposite side)
        depth_image = depth_image[: , self.invalid_band_size : IMG_WIDTH  - self.invalid_band_size]

        # Take middle slice of image
        middle_depth = depth_image[(int)(IMG_HEIGHT / 2) - 10 : (int)(IMG_HEIGHT / 2) + 10, :]
        middle_depth_averages = np.mean(middle_depth, axis=0)
        self.running_average.addTerm(middle_depth_averages)

    def findGap(self):

        middle_depth_filtered = self.running_average.average()
        print("Middle depth filtered Alternate:")
        print(middle_depth_filtered)

        # Find largest gap above depth ceiling
        ceiling = self.ceiling_m / self.depth_frame_units  # in RealSense depth units

        # get black/white image
        middle_depth_bw = np.empty_like(middle_depth_filtered)
        for i in range(0, np.size(middle_depth_filtered)):
            if middle_depth_filtered[i] > ceiling:
                middle_depth_bw[i] = 1
            else:
                middle_depth_bw[i] = 0


        # mean filter
        averageLength = 9
        for i in range(0, np.size(middle_depth_bw)):
            if i > averageLength and np.size(middle_depth_bw) - i - 1 > averageLength:
                newVal = np.sum(middle_depth_bw[i - averageLength : i + averageLength + 1]) / (2 * averageLength + 1)

                newVal = round(newVal)

                middle_depth_bw[i] = newVal

        print("Middle BW alternate:")
        print(middle_depth_bw)

        # Find biggest gap
        count = 0
        longest = -1
        longestStart = -1
        longestEnd = -1
        for i in range(0, np.size(middle_depth_bw)):
            if middle_depth_bw[i] == 1:
                count += 1
            else:
                if count > longest:
                    longest = count
                    longestEnd = i - 1
                    longestStart = longestEnd - count
                    count = 0

        # Corner case for when the gap reaches the side
        if count > longest:
            longest = count
            longestEnd = np.size(middle_depth_bw)
            longestStart = longestEnd - count
            count = 0

        gapCenter = (int)((longestStart + longestEnd) / 2)

        return gapCenter, longest



class RunningAverage:

    def __init__(self, size):
        self.max_length = size
        self.length = 0
        self.first = None
        self.last = None
    
    def addTerm(self, term):
        if self.first == None:
            self.first = Node(term, None)
            self.last = self.first
        
        else:
            new = Node(term, None)
            self.last.next = new
            self.last = new
            self.length += 1

            if self.length > self.max_length:
                self.first = self.first.next
                self.length += -1
    
    def sum(self):
        current = self.first
        sum = np.zeros(np.shape(current.data))
        while current != None:
            sum = sum + current.data
            current = current.next
        print("middle depth sum alternate")
        print(sum)
        return sum
    
    def average(self):
        return self.sum()/self.length



class Node:
    def __init__(self, data, next):
        self.data = data
        self.next = next
    

    

