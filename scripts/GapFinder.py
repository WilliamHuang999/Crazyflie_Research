
import numpy as np

class GapFinder:

    def __init__(self, ceiling_m = 2,  running_average_length = 10):
        self.ceiling_m = ceiling_m
        self.running_average = RunningAverage(size = running_average_length)
        self.depth_frame_units = -1
    
    def addFrame(self, depth_image):
        (IMG_HEIGHT, IMG_WIDTH) = np.shape(depth_image)

        # Take middle slice of image
        middle_depth = depth_image[(int)(IMG_HEIGHT / 2) - 10 : (int)(IMG_HEIGHT / 2) + 10, :]
        middle_depth_averages = np.mean(middle_depth, axis=0)
        self.running_average.addTerm(middle_depth_averages)
    
    def set_depth_frame_units(self, depth_frame_units):
        self.depth_frame_units = depth_frame_units

    def findGap(self):

        middle_depth_filtered = self.running_average.average()
        print(middle_depth_filtered)

        # Find largest gap above depth ceiling
        ceiling = self.ceiling_m / self.depth_frame_units  # in RealSense depth units
        print(ceiling)
        print(self.ceiling_m / self.depth_frame_units)
        print(self.depth_frame_units)
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
            self.length += 1
        
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
        return sum
    
    def average(self):
        return self.sum()/self.length



class Node:
    def __init__(self, data, next):
        self.data = data
        self.next = next
    

    

