#!/usr/local/opt/python/bin/python2.7

import serial, sys
import math
import numpy
import matplotlib.pyplot as plt
import csv
import matplotlib.patches as mpatches

DMA_l = (60)/2
frame_size = 62*DMA_l;
bytes_per_sample = 2;

if __name__ == "__main__":
    frames = [];

    usbfriend = serial.Serial(sys.argv[1]);
    
    usbfriend.write('x');
    
    for frame_idx in xrange(1):
        data = usbfriend.read(frame_size * bytes_per_sample);
        f = open('log.txt', 'w');
        f.write(data);
        f.close();

        frame = [];

        for byte_idx in xrange(0, len(data), 2):
            lsb = ord(data[byte_idx]);
            msb = ord(data[byte_idx+1]);
            val = (msb << 8) | lsb;
            # TODO: add two's complement logic
            frame.append(val);
        
        frames.append(frame);
    
    usbfriend.close();
    
    print frames
    
    plt.xlabel("Samples");
    plt.ylabel("Signal");
    plt.plot(frames[0]);
    plt.show();
