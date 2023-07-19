import numpy as np
from matplotlib import pyplot as plt
import time

x, y, z = [], [], []
ts = []

with open('./imu.csv', 'r') as f:
    lines = f.readlines()[1:]
f.close()

for line in lines:
    # print(line)
    items = line.split(',')
    timestamp = float(items[0])/1e9
    ax = float(items[4])
    ay = float(items[5])
    az = float(items[6])

    ts.append(timestamp)
    x.append(ax)
    y.append(ay)
    z.append(az)
    
    plt.ion()
    plt.subplot(311)
    plt.plot(ts, x, 'r-', label='x')
    plt.xlabel('time')
    plt.ylabel('linear acceleration x')

    plt.subplot(312)
    plt.plot(ts, y, 'g-', label='y')
    plt.xlabel('time')
    plt.ylabel('linear acceleration y')

    plt.subplot(313)
    plt.plot(ts, z, 'b-', label='z')
    plt.xlabel('time')
    plt.ylabel('linear acceleration z')

    plt.pause(0.000001)
    plt.ioff()

    if len(ts) > 25:
        ts.pop(0)
        x.pop(0)
        y.pop(0)
        z.pop(0)

