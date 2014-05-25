#!/usr/local/opt/python/bin/python2.7

import serial, sys
import math
import numpy
import matplotlib.pyplot as plt
import csv
import matplotlib.patches as mpatches

if __name__ == "__main__":
    arrayfriend = [];

    usbfriend = serial.Serial(sys.argv[1]);

    usbfriend.write('x');

    for i in xrange(int(sys.argv[2])):
        num = int(usbfriend.readline().strip(), 10);
        arrayfriend.append(num);

    usbfriend.close();

    print arrayfriend

    plt.xlabel("Samples");
    plt.ylabel("Signal");
    plt.plot(arrayfriend);
    plt.show();

    print arrayfriend;
