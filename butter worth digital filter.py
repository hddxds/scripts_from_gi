import os
import numpy as np
from scipy.signal import butter, lfilter, freqz
import matplotlib.pyplot as plt

def read(path, column=0):
    file = open(path).readlines()
    values = []
    times = []
    for line in file[1:]:
        value  = line.split(',')[column]
        time = line.split(',')[0]
        values.append(float(value))
        times.append(float(time))
    print(values)
    print(np.shape(values))
    return values, times



def butter_lowpass(cutoff, fs, order=5):
    #nyquist is half than that of sampling freq
    nyq = 0.5 * fs

    # normalize cut-off frequency
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)

    #return denomiter and numerator of the filer
    return b, a

def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y



if __name__ == '__main__':
    data, times = read('/home/gishr/ulog/19_23_03(1)_actuator_controls_0_0.csv', 2)
    time_gap = (times[-1] - times[0])/10000000
    print("time gap is : ", time_gap, 's')

    order = 6
    fs = 100
    cutoff = 3.7

    #define the filter given cutoff freq and fs
    #the selection of cutoff is arbitary
    b, a = butter_lowpass(cutoff, fs, order)

    T = time_gap  # seconds
    n = len(data)  # total number of samples
    t = np.linspace(0, T, n, endpoint=False)

    # Filter the data, and plot both the original and filtered signals.
    y = butter_lowpass_filter(data, cutoff, fs, order)

    plt.plot()
    plt.plot(t, data, 'b-', label='data')
    plt.plot(t, y, 'g-', linewidth=2, label='filtered data')
    plt.xlabel('Time [sec]')
    plt.grid()
    plt.legend()

    plt.subplots_adjust(hspace=0.35)
    plt.show()