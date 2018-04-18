#!/usr/bin/python

class MovingAverage(object):
    
    def __init__(self):
        # A list of floating point numbers.
        self._sampleBuffer = []
        self._bufferSize = 0
        self._currentSample = 0
        self._sampleCount = 0
        return
    
    
    def addSample(self, sample):
        self._sampleBuffer[self._currentSample] = float(sample)
        print 'Add Sample at index:', self._currentSample
        self._currentSample = (self._currentSample + 1) % self._bufferSize
        # Increment the sample count until it equals the buffer size:
        if self._sampleCount < self._bufferSize:
            self._sampleCount += 1
        
        print 'Add Sample, count:', self._sampleCount
        return
    
    
    # virtual float computeAverage(void) = 0;

class CumulativeMovingAverage(MovingAverage):
    pass


class WeightedMovingAverage(MovingAverage):
    
    def __init__(self, bufferSize):
        MovingAverage.__init__(self)
        self._bufferSize = bufferSize
        return
    
    
    def computeAverage(self):
        average = 0.0
        multSum = 0
        index = self._currentSample - 1
        if index < 0:
            index = self._sampleCount - 1
        
        for i in range(self._sampleCount):
            average += self._sampleBuffer[index] * (self._sampleCount - i)
            # Work from the most recent sample to the oldest sample.
            index = index - 1
            if index < 0:
                index = self._sampleCount - 1
            
            multSum += self._sampleCount - i
        
        return average / multSum
    


class ExponentialMovingAverage(MovingAverage):
    """
    The coefficient a represents the degree of weighting decrease, 
    a constant smoothing factor between 0 and 1. A higher a 
    discounts older observations faster.
    """
    
    def __init__(self, alpha=None):
        MovingAverage.__init__(self)
        
        if alpha is None:
            self.__alpha = 0.5
        else:
            self.__alpha = alpha
        
        self.__average = 0.0
        return
    
    
    def addSample(self, sample):
        if self._sampleCount == 0:
            self._sampleCount += 1
            self.__average = sample
        else:
            self.__average = self.__alpha * sample + (1.0 - self.__alpha) \
                * self.__average;
            
        return
    
    
    def setAlpha(self, alpha):
        self.__alpha = alpha
        return

    def computeAverage(self):
        return self.__average
    


if __name__ == "__main__":
    avg = ExponentialMovingAverage(0.99)
    avg.addSample(1)
    avg.addSample(2)
    avg.addSample(3)
    avg.addSample(4)
    avg.addSample(5)
    print avg.computeAverage()
    avg.setAlpha(0.01)
    avg.addSample(4)
    avg.addSample(3)
    avg.addSample(2)
    avg.addSample(1)
    avg.addSample(0)
    print avg.computeAverage()



