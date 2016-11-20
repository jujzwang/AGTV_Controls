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
                     int, float,int, float,int, float,float)
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
        accel_calibrations = {
            "-x": "vertically with the habitat up and the main motor down",
            "+x": "vertically with the habitat down and the main motor up",
            "-y": "on its side with the power switch pointing up",
            "+y": "on its side with the power switch pointing down",
            "-z": "upside down, resting on the main and slewing motor cases",
            "+z": "right side up, with the Arduino facing down",
        }

        axis_indices = {"-x": 0, "+x": 0, "-y": 1, "+y": 1, "-z": 2, "+z": 2}

        measured_gravity = {"local": {}, "remote": {}}

        for axis in accel_calibrations.keys():
            print "Position the apparatus with gravity along the %s axis" % axis
            print "(%s)" % accel_calibrations[axis]
            print "Press Enter when ready:",
            while True:
                (readable, _w, _x) = select.select((0, ser), (), ())
                if 0 in readable:
                    sys.stdin.readline()
                    break
                (ok, data) = read_messages(ser)
                if len(data) > 0:
                    print "      (%.3f, %.3f)      " % (
                        data[-1]['local'][axis_indices[axis]],
                        data[-1]['remote'][axis_indices[axis]]),
                    print "\rPress Enter when ready:",
            start = datetime.now()
            lvalues = []
            rvalues = []
            while True:
                elapsed = (datetime.now() - start).total_seconds()
                if elapsed > 10:
                    break
                print "\rCalibrating...         ",
                (ok, data) = read_messages(ser)
                for line in data:
                    lvalues.append(line['local'][axis_indices[axis]])
                    rvalues.append(line['remote'][axis_indices[axis]])
                    print "      (%.3f, %.3f)      " % (lvalues[-1], rvalues[-1]),
                    print "\rCalibrating...         ",
            measured_gravity['local'][axis] = sum(lvalues) / len(lvalues)
            measured_gravity['remote'][axis] = sum(rvalues) / len(rvalues)
            print "     done.\n"

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
        degrees_per_turn = {}

        (angle_sums, duration) = get_angle_sums(
            "Leave the apparatus perfectly still.", 60)
        for naxis in xrange(3):
            drift_rates[naxis] = angle_sums[naxis] / duration

        gyro_axes = {"+x": "hab up, main motor down, spin left",
                     "-x": "hab up, main motor down, spin right",
                     "+y": "main motor on left with flywheels perpendicular to floor, hab on right, slewing motor away from you, spin left",
                     "-y": "main motor on left with flywheels perpendicular to floor, hab on right, slewing motor away from you, spin right",
                     "+z": "hold in normal position with the slewing motor above the truss, spin right",
                     "-z": "hold in normal position with the slewing motor above the truss, spin left"}
        for axis in gyro_axes.keys():
            (angle_sums, duration) = get_angle_sums(
                "Rotate the apparatus through a full 360 degrees along the\n" +
                "%s axis (%s)." % (axis, gyro_axes[axis]))
            naxis = axis_indices[axis]
            degrees_per_turn[axis] = angle_sums[naxis] - (drift_rates[naxis] * duration)

        print "Calibration done; resetting."
        ser.write("#z;")
    
    finally:    
        def print_data(d):
            print "    +x %8.5f     -x %8.5f" % (d['+x'], d['-x'])
            print "    +y %8.5f     -y %8.5f" % (d['+y'], d['-y'])
            print "    +z %8.5f     -z %8.5f" % (d['+z'], d['-z'])

        print
        print "Accelerometers at 1 G:"
        print "  Local:"
        print_data(measured_gravity['local'])
        print "  Remote:"
        print_data(measured_gravity['remote'])
        print
        # +/- 1 = (read - bias) * scale
        # +1/scale = pread - bias
        # -1/scale = mread - bias
        # -: 0 = (pread + mread) - 2*bias  --> bias = (pread + mread)/2
        # +: 2/scale = pread - mread       --> scale = 2/(pread - mread)
        bias = lambda pv, mv: (pv + mv)/2.0
        scale = lambda pv, mv: 2.0/(pv - mv)
        print "local_accel:"
        d = measured_gravity['local']
        # - signs because gravity is opposite acceleration
        print "  .bias = {%.5f, %.5f, %.5f}," % (
            bias(d['+y'], d['-y']), bias(d['+x'], d['-x']), bias(-d['-z'], -d['+z']))
        print "  .scale = {%.5f, %.5f, %.5f}," % (
            -scale(d['+y'], d['-y']), -scale(d['+x'], d['-x']), -scale(-d['-z'], -d['+z']))
        print "remote_accel:"
        d = measured_gravity['remote']
        print "  .bias = {%.5f, %.5f, %.5f}," % (
            bias(d['+x'], d['-x']), bias(d['+z'], d['-z']), bias(-d['-y'], -d['+y']))
        print "  .scale = {%.5f, %.5f, %.5f}," % (
            -scale(d['+x'], d['-x']), -scale(d['+z'], d['-z']), -scale(-d['-y'], -d['+y']))
        print
        print
        print "Gyro:"
        print "  Drift rates (dps):"
        print "     x %8.5f" % drift_rates[0]
        print "     y %8.5f" % drift_rates[1]
        print "     z %8.5f" % drift_rates[2]
        print "  Degrees per turn:"
        print_data(degrees_per_turn)
        print
        print "gyro:"
        print "  .bias = {%.5f, %.5f, %.5f}," % (
            drift_rates[0], -drift_rates[1], -drift_rates[2])
        d = degrees_per_turn
        print "  .scale = {%.5f, %.5f, %.5f}," % (
            360.0 * scale(d['+x'], d['-x']),
            360.0 * scale(-d['-y'], -d['+y']),
            360.0 * scale(-d['-z'], -d['+z']))
        print
        print "Goodbye."
        sleep(1)
        read_messages(ser)
        ser.close()

if __name__ == "__main__":
    sys.exit(main())
