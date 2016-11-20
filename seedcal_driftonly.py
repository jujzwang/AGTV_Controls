#!/usr/bin/python

import os, sys, re
from datetime import *
import serial
import select
from time import sleep

serial_buffer = ""
def read_messages(ser):
    global serial_buffer
    ok_lines = []
    data_lines = []
    while True:
        ret = ser.read(100)
        if ret == "":
            break;
        serial_buffer += ret
    while ";" in serial_buffer:
        semi = serial_buffer.index(";")
        line = serial_buffer[:semi]
        serial_buffer = serial_buffer[(semi+1):]
        if "#" not in line:
            continue
        hash = line.index("#")
        line = line[hash:]
        words = line.split()
        if words[0] == "#ERR":
            print "Arduino reported error: %s" % words[1:]
        elif words[0] == "#OK":
            print " ".join(words)
            ok_lines.append((words[2], int(words[1])))
        elif words[0] == "#DATA":
            types = (int, int,
                     float,float,float, float,float,float,float, float,float,float,
                     float,float,float, float,float,float,
                     float,float,float, float,float,float,
                     float,
                     int, float,int, float,int)
            params = [types[i](words[i+1]) for i in xrange(len(words)-1)]
            gyro_rates = params[2:5]
            gyro_angles = params[9:12]
            laccel = params[12:15]
            raccel = params[18:21]
            data_lines.append({"time": params[0],
                               "rate": gyro_rates, "angle": gyro_angles,
                               "local": laccel, "remote": raccel})

    return (ok_lines, data_lines)

def read_until(ser, thing_ok, n_data, timeout=None):
    now = datetime.now()
    all_ok = []
    all_data = []
    while True:
        if (timeout is not None and
            (datetime.now() - now).total_seconds() > timeout):
            return (all_ok, all_data, False)
        (ok, data) = read_messages(ser)
        all_ok += ok
        all_data += data
        if ((thing_ok is None or thing_ok in [l[0] for l in all_ok]) and
            len(all_data) >= n_data):
            return (all_ok, all_data, True)

def main():
    # disable stdout buffering
    sys.stdout = os.fdopen(sys.stdout.fileno(), 'w', 0)

    ser = serial.Serial("/dev/cu.RN42-5419-SPP", 115200, timeout=0)
    read_messages(ser)
    ser.write("#lc;#d1;#r0;#C;")
    (ok, data, completed) = read_until(ser, "calibrate", 0, 10)
    if not completed:
        print "Could not start calibration"
        return 1

    try:
        def get_angle_sums(instruction, duration=None):
            # Leave the Arduino with incomplete commands so it doesn't send us data
            ser.write("#")
            print instruction
            print "Press Enter when ready:",
            sys.stdin.readline()
            ser.write("a;#")
            (ok, data, complete) = read_until(ser, "newangle", 0, 5)
            if not complete:
                print "Could not start gyro calibration"
                return 1
            start_millis = ok[-1][1]
            if duration is not None:
                for i in xrange(duration):
                    print "\r% 4d / %d" % (i, duration)
                    sleep(1)
            else:
                print "Press Enter when done:",
                sys.stdin.readline()
            ser.write("p;")
            (ok, data, complete) = read_until(ser, None, 1, 5)
            if not complete:
                print "Could not get gyro data"
                return 1
            end_millis = data[-1]['time']
            print "   done.\n"
            return (data[-1]['angle'], (end_millis - start_millis) / 1000.0)

        drift_rates = [0,0,0]

        (angle_sums, duration) = get_angle_sums(
            "Leave the apparatus perfectly still.", 60)
        for naxis in xrange(3):
            drift_rates[naxis] = angle_sums[naxis] / duration

        print "Calibration done; resetting."
        ser.write("#z;")
    
    finally:    
        print "Gyro:"
        print "  Drift rates (dps):"
        print "     x %8.5f" % drift_rates[0]
        print "     y %8.5f" % drift_rates[1]
        print "     z %8.5f" % drift_rates[2]
        print
        print "gyro:"
        print "  .bias = {%.5f, %.5f, %.5f}," % (
            drift_rates[0], -drift_rates[1], -drift_rates[2])
        print
        print "Goodbye."
        sleep(1)
        read_messages(ser)
        ser.close()

if __name__ == "__main__":
    sys.exit(main())
