import serial
from datetime import *

ser = serial.Serial('/dev/cu.RN42-5419-SPP',115200,timeout=0)
ser.write('#lr;#dl;#r0;#pv;')
from Tkinter import *

class App:
  def __init__(self, master):
    self.lastPing = datetime.now()
    self.master = master
    self.periodicCall()
    frame = Frame(master)
    frame.pack()
    master.title("Motor controls")
    #button 1
    self.button1 = Button(frame,
                         text="Connect",
                         command=self.enable)
    self.button1.pack(side=LEFT)

    #button 2
    self.button2 = Button(frame,
                         text="run motors",
                         command=self.runn)
    self.button2.pack(side=LEFT)

##    #button 3
##    self.button3 = Button(frame,
##                         text="runn",
##                         command=self.runn(.1,0))
##    self.button3.pack(side=LEFT)

    #text input 1
    self.label1 = Label(frame, text = "Main Motor Throttle")
    self.label1.pack(side=LEFT)
#    self.text1 = Entry(frame,textvariable=content)
    self.text1 = Entry(frame)
    self.text1.pack(side=LEFT)

    #text input 2
    self.label2 = Label(frame, text = "Slew Motor Throttle")
    self.label2.pack(side=LEFT)
    self.text2 = Entry(frame)
    self.text2.pack(side=LEFT)

    #button 4
    self.button4 = Button(frame,
                         text="Kill",
                         command=self.kill)
    self.button4.pack(side=LEFT)
  def kill(self):
    ser.write('#r0;')
    ser.close()
    print "you have killed the program"

  def enable(self):
    ser.write('#r1;')
    print "you are now connected"


  def motor0(self,throttle0): # throttle0 can be any float between 0 and 1
    print ('#t0=%.3f;' % float(throttle0)) 
    ser.write('#t0=%.3f;' % float(throttle0))
    self.receive()

  def motor1(self,throttle1): # throttle1 can be any float between -1 and 1
    print "motor one set"
    ser.write('#t1=%.3f;' % float(throttle1))
    self.receive()


  def receive(self):
    msg = ""
    while True:
      ret = ser.read(100)
      if ret == "":
          break;
      msg += ret
    print msg
    
  def runn(self):
    main = .1
    slew = 0
    #main = float(entries[])
    #put code here
    print "I cliked runn"
    #print content.get()
    self.motor0(main)
    #self.motor1(slew)

 
  def periodicCall(self):
      now = datetime.now()

      if (now - self.lastPing).total_seconds() > .3:
          print "Time to ping!"
          ser.write("#p;");
          self.lastPing = now;
      self.master.after(50, self.periodicCall); 
      

root = Tk()
app = App(root)
root.mainloop()


#main = float(raw_input("main throttle"))
#slew = float(raw_input("slew throttle"))

