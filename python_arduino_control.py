import serial
ser = serial.Serial('/dev/cu.RN42-5419-SPP',115200,timeout=0)
ser.write('#lr;#dl;#r0;#pv;')


def enable():
    ser.write('#r1;')


def motor0(throttle0): # throttle0 can be any float between 0 and 1 
    ser.write('#t0=%.3f;' % float(throttle0)) 

def motor1(throttle1): # throttle1 can be any float between -1 and 1
    ser.write('#t1=%.3f;' % float(throttle1))



def kill():
    ser.write('#r0')
    ser.close()


def runn():
    #put code here
    motor0()
    motor1()
    kill()          
