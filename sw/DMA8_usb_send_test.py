#!/usr/local/opt/python/bin/python2.7

import serial, sys
import math
import numpy
import matplotlib.pyplot as plt
import csv
import matplotlib.patches as mpatches

gels = 5;
frame_size = gels*62*100;
bytes_per_sample = 1;

def tc8(val):
    out = 0;
    if (val & 0x80):
        out = (~val & 0x7F) + 1;
        out = -1 * out;
    else:
        out = val;
    return out;

def csv_read(a):
    x = []
    x_final = []
    x = open(a+".csv","U")
    x_reader = csv.reader(x, delimiter=' ', quotechar='|')
    for row in x_reader:
        x_final.append(float(row[0]))
    
    return x_final



if __name__ == "__main__":
    frames = [];

    usbfriend = serial.Serial(sys.argv[1]);
    
    #usbfriend.write('x');
    
    for frame_idx in xrange(1):
        data = usbfriend.read(frame_size * bytes_per_sample);
        f = open('log.txt', 'w');
        f.write(data);
        f.close();

        frame = [];


        for byte_idx in xrange(len(data)):
            val = ord(data[byte_idx]);
            frame.append(val);


        """
        for byte_idx in xrange(0, len(data), 2):
            lsb = ord(data[byte_idx]);
            msb = ord(data[byte_idx+1]);
            val = (msb << 8) | lsb;
            # TODO: add two's complement logic
            frame.append(val);
        """
        
        frames.append(frame);
    
    usbfriend.close();
    
    for x in range(len(frames)):
        with open(str(x)+'data.csv', 'wb') as csvfile:
            spamwriter = csv.writer(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
            for y in range(len(frames[x])):
                spamwriter.writerow([frames[x][y]])

    #print frames
    
    plt.xlabel("Samples");
    plt.ylabel("Signal");
    plt.plot(frames[0]);
    #plt.plot(frames[0][:100]);
    plt.show();
