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

def tc12(val):
    out = 0;
    if (val & 0x0800):
        out = (~val & 0x7FF) + 1;
        out = -1 * out;
    else:
        out = val;
    return out;

if __name__ == "__main__":
    frames = [];
    
    for frame_idx in xrange(1):
        f = open('log.txt', 'r');
        data = f.read();
        f.close();

        frame = [];

        for byte_idx in xrange(0, len(data), 2):
            lsb = ord(data[byte_idx]);
            msb = ord(data[byte_idx+1]);
            val = (msb << 8) | lsb;
            # TODO: add two's complement logic
            frame.append(tc12(val));
        
        frames.append(frame);
    
    print frames
    
    plt.xlabel("Samples");
    plt.ylabel("Signal");
    plt.plot(frames[0]);
    plt.show();
