#!/usr/bin/python

import os, sys, re
from datetime import *
import math
import serial
import collections
from time import sleep
import traceback

from Tkinter import *
from ttk import *
import tkMessageBox

class MasterControl(Frame):
    def __init__(self, master):
        Frame.__init__(self, master)
        s = Style()
        s.configure("Big.TButton", font=('Helvetica', 18, 'bold'))
        s.map("TButton", foreground=[('disabled', 'gray')])
        self.connectLabel = StringVar()
        self.armLabel = StringVar()
        self.createWidgets()

    def createWidgets(self):
        self.connect = Button(self, textvariable=self.connectLabel, style="Big.TButton", command=self.master.doConnect)
        self.arm = Button(self, textvariable=self.armLabel, style="Big.TButton", command=self.master.doArm)
        self.idle = Button(self, text="Idle", style="Big.TButton", command=self.master.doIdle)
        self.connect.grid(column=0, row=0, sticky=E+W)
        self.arm.grid(column=1, row=0, sticky=E+W)
        self.idle.grid(column=2, row=0, sticky=E+W)
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)
        self.columnconfigure(2, weight=1)

    def setState(self, state):
        if state == "disconnected":
            self.connectLabel.set("Connect")
            self.armLabel.set("Arm")
            self.connect.state(["!disabled"])
            self.arm.state(["disabled"])
            self.idle.state(["disabled"])
        elif state == "disabled":
            self.connectLabel.set("Disconnect")
            self.armLabel.set("Arm")
            self.connect.state(["!disabled"])
            self.arm.state(["!disabled"])
            self.idle.state(["disabled"])
        elif state == "armed":
            self.connectLabel.set("Disconnect")
            self.armLabel.set("Kill")
            self.connect.state(["disabled"])
            self.arm.state(["!disabled"])
            self.idle.state(["!disabled"])
        else:
            raise RuntimeException, "invalid state"

# Each thing in format is a string [for label] or
# a tuple (name, type, args) [for input].
# (name, 'entry', 'fn') is an entry validated by applying 'fn'
# to the input - OK if it doesn't throw an exception.
# (name, 'choice', ['a', 'b']) is a readonly combobox choosing
# between 'a' and 'b'.
# The empty string will be rendered as a newline.
class ControlItem(Frame):
    def __init__(self, master, typeState, typeValue, format):
        Frame.__init__(self, master)
        validateCommand = self.register(self.validate)
        self.command = dict()
        self.widget = dict()
        self.validator = dict()

        for item in format:
            if isinstance(item, tuple):
                self.command[item[0]] = StringVar()

        self.button = Radiobutton(self, variable=typeState,
                                  value=typeValue, takefocus=False)
        self.button.grid(row=0, column=0)

        format = [""] + list(format)
        nextrow = 0
        nextcol = 1
        frame = None
        for item in format:
            if isinstance(item, str):
                if item == "":
                    frame = Frame(self)
                    frame.grid(row=nextrow, column=1, sticky=W)
                    nextrow += 1
                    nextcol = 0
                else:
                    l = Label(frame, text=item)
                    l.bind("<1>", lambda e: self.button.invoke())
                    l.grid(row=0, column=nextcol)
                    nextcol += 1
            elif item[1] == "entry":
                n = item[0]
                self.validator[n] = item[2]
                self.widget[n] = Entry(frame, name=n,
                                       textvariable=self.command[n],
                                       validate='key', width=5,
                                       validatecommand=(validateCommand, '%W', '%P'))
                self.widget[n].bind("<FocusIn>", lambda e, n=n: self.enter(n))
                self.widget[n].bind("<FocusOut>",
                                    lambda e: self.widget[n].select_clear())
                self.widget[n].bind("<Return>", lambda e: self.master.go.invoke())
                self.widget[n].grid(row=0, column=nextcol)
                nextcol += 1
            elif item[1] == "choice":
                n = item[0]
                self.widget[n] = Combobox(frame, name=n, width=10,
                                          textvariable=self.command[n],
                                          values=list(item[2]))
                self.widget[n].state(["readonly"])
                self.widget[n].current(0)
                self.widget[n].bind("<FocusIn>", lambda e: self.button.invoke())
                self.widget[n].bind("<FocusOut>",
                                    lambda e: self.widget[n].select_clear())
                self.widget[n].grid(row=0, column=nextcol)
                nextcol += 1
            else:
                raise ValueError, "unknown widget type"

    def enter(self, name):
        self.button.invoke()
        self.widget[name].select_range(0, END)

    def validate(self, name, proposed):
        n = name[name.rindex(".")+1:]
        if proposed == "" or proposed == "-" or proposed == ".":
            return True
        try:
            t = self.validator[n]
            t(proposed)
            return True
        except ValueError:
            return False

class ControlLoopConfig(Frame):
    def __init__(self, master, code, def_kp, def_ki, def_imax):
        Frame.__init__(self, master)
        self.kp = StringVar()
        self.ki = StringVar()
        self.i_max = StringVar()
        self.kp.set(str(def_kp))
        self.ki.set(str(def_ki))
        self.i_max.set(str(def_imax))
        self.code = code
        self.createWidgets()

    def createWidgets(self):
        Label(self, text="kp").grid(row=0, column=0)
        self.kp_entry = Entry(self, textvariable=self.kp, width=8)
        self.kp_entry.grid(row=0, column=1, sticky=E+W)
        Label(self, text="ki").grid(row=0, column=2)
        self.ki_entry = Entry(self, textvariable=self.ki, width=8)
        self.ki_entry.grid(row=0, column=3, sticky=E+W)
        Label(self, text="lim").grid(row=0, column=4)
        self.lim_entry = Entry(self, textvariable=self.i_max, width=8)
        self.lim_entry.grid(row=0, column=5, sticky=E+W)

        for widget in (self.kp_entry, self.ki_entry, self.lim_entry):
            widget.bind("<FocusIn>", lambda e, w=widget: w.select_range(0, END))
            widget.bind("<FocusOut>", lambda e, w=widget: w.select_clear())
        self.kp_entry.bind("<Return>", lambda e: self.master.master.master.ser.write(
                "#L%sp%.7f;" % (self.code, float(self.kp.get()))))
        self.ki_entry.bind("<Return>", lambda e: self.master.master.master.ser.write(
                "#L%si%.7f;" % (self.code, float(self.ki.get()))))
        self.lim_entry.bind("<Return>", lambda e: self.master.master.master.ser.write(
                "#L%sl%.7f;" % (self.code, float(self.i_max.get()))))

class SpinControl(LabelFrame):
    def __init__(self, master):
        LabelFrame.__init__(self, master, text="Spinning")
        self.typeState = StringVar()
        self.createWidgets()

    def createWidgets(self):
        self.throttle = ControlItem(self, self.typeState, "throttle",
                                    ("Throttle setting of",
                                     ("v", "entry", float),
                                     "(0 to 1)"))
        self.rpm = ControlItem(self, self.typeState, "rpm",
                               ("Speed setting of",
                                ("v", "entry", int),
                                "(RPM; ~2000 to ~10000)"))
        self.accel = ControlItem(self, self.typeState, "accel",
                                 ("Acceleration of",
                                  ("v", "entry", float),
                                  "(G; 0 to ~0.4) of"))
        self.go = Button(self, text="Go", command=self.doGo)

        self.throttle.button.invoke()
        self.throttle.grid(row=0, column=0, sticky=W)
        self.rpm.grid(row=1, column=0, sticky=W)
        self.accel.grid(row=2, column=0, sticky=W)
        self.go.grid(row=3, column=0, sticky=E+W)

        loop_config_frame = Frame(self)
        loop_config_frame.grid(row=4, column=0, sticky=E+W)
        Label(loop_config_frame, text="RPM:").grid(row=0, column=0, sticky=E)
        Label(loop_config_frame, text="Accel:").grid(row=1, column=0, sticky=E)
        ControlLoopConfig(loop_config_frame, "R", 0.00003, 0.00005, 6000).grid(
            row=0, column=1, sticky=E+W)
        ControlLoopConfig(loop_config_frame, "G", 1e4, 9e3, 0.6).grid(
            row=1, column=1, sticky=E+W)
        loop_config_frame.columnconfigure(1, weight=1)

    def doGo(self):
        print "doGO"
        if self.typeState.get() == "throttle":
            if float(self.throttle.command['v'].get()) > 1:
                print "Cant see me"
                self.throttle.command['v'].set(
                    "%.2f" % (float(self.throttle.command['v'].get()) / 100.0))
            print "about to set throttle"
            print("#t0=%.3f;" % float(self.throttle.command['v'].get()))
            self.master.ser.write("#t0=%.3f;" %
                                  float(self.throttle.command['v'].get()))
        elif self.typeState.get() == "rpm":
            if int(self.rpm.command['v'].get()) < 100:
                self.rpm.command['v'].set(
                    "%d" % (int(self.rpm.command['v'].get()) * 100))
            self.master.ser.write("#R%d;" % int(self.rpm.command['v'].get()))
        elif self.typeState.get() == "accel":
            if float(self.accel.command['v'].get()) > 1:
                self.accel.command['v'].set(
                    "%.2f" % (float(self.accel.command['v'].get()) / 100.0))
            self.master.ser.write("#G%.3f;" % float(self.accel.command['v'].get()))

class SlewControl(LabelFrame):
    def __init__(self, master):
        LabelFrame.__init__(self, master, text="Slewing")
        self.typeState = StringVar()
        self.createWidgets()

    def createWidgets(self):
        self.throttle = ControlItem(self, self.typeState, "throttle",
                                    ("Throttle setting of",
                                     ("v", "entry", float),
                                     "(-1 to 1)"))
        self.impulse = ControlItem(self, self.typeState, "impulse",
                                   ("Impulse of magnitude",
                                    ("mag", "entry", float),
                                    "(-1 to 1),", "",
                                    "duration",
                                    ("dur", "entry", int),
                                    "ms, shape",
                                    ("shape", "choice",
                                     ["square", "triangular",
                                      "sawtooth", "sine"])))
        self.duty = ControlItem(self, self.typeState, "duty",
                                ("Slew at mag",
                                 ("mag", "entry", float),
                                 "(-1 to 1) for",
                                 ("width", "entry", float),
                                 "(0 to 1)", "",
                                 "of each half-cycle, centered",
                                 ("center", "entry", float),
                                 "(0 to 1)", "",
                                 "through the half-cycle, shape",
                                 ("shape", "choice",
                                  ["square", "triangular", "sine"]), ",", "",
                                 "and on negative half-cycles",
                                 ("behavior", "choice",
                                  ["idle", "reverse", "forward"])))
        self.go = Button(self, text="Go", command=self.doGo)

        self.throttle.button.invoke()
        self.throttle.grid(row=0, column=0, sticky=W)
        self.impulse.grid(row=1, column=0, sticky=W)
        self.duty.grid(row=2, column=0, sticky=W)
        self.go.grid(row=3, column=0, sticky=E+W)

    def doGo(self):
        slew_shape_char = {"square": 'q', "triangular": 't', "sawtooth": '/',
                          "sine": 's'}
        duty_type_char = {"idle": '+', "reverse": '*', "forward": '-'}

        if self.typeState.get() == "throttle":
            if abs(float(self.throttle.command['v'].get())) > 1:
                self.throttle.command['v'].set(
                    "%.2f" % (float(self.throttle.command['v'].get()) / 100.0))
            self.master.ser.write("#t1=%.3f;" %
                                  float(self.throttle.command['v'].get()))
        elif self.typeState.get() == "impulse":
            if abs(float(self.impulse.command['mag'].get())) > 1:
                self.impulse.command['mag'].set(
                    "%.2f" % (float(self.impulse.command['mag'].get()) / 100.0))
            self.master.ser.write("#I%c%.3f %d;" % (
                    slew_shape_char[self.impulse.command['shape'].get()],
                    float(self.impulse.command['mag'].get()),
                    int(self.impulse.command['dur'].get())))
        elif self.typeState.get() == "duty":
            if abs(float(self.duty.command['mag'].get())) > 1:
                self.duty.command['mag'].set(
                    "%.2f" % (float(self.duty.command['mag'].get()) / 100.0))
            if float(self.duty.command['center'].get()) > 1:
                self.duty.command['center'].set(
                    "%.2f" % (float(self.duty.command['center'].get()) / 100.0))
            if float(self.duty.command['width'].get()) > 1:
                self.duty.command['width'].set(
                    "%.2f" % (float(self.duty.command['width'].get()) / 100.0))
            self.master.ser.write("#D%c%c%.3f %.3f,%.3f;" % (
                    duty_type_char[self.duty.command['behavior'].get()],
                    slew_shape_char[self.duty.command['shape'].get()],
                    float(self.duty.command['mag'].get()),
                    float(self.duty.command['center'].get()),
                    float(self.duty.command['width'].get())))

class Status(LabelFrame):
    def __init__(self, master):
        LabelFrame.__init__(self, master, text="Status")
        self.spin_throttle = StringVar()
        self.slew_throttle = StringVar()
        self.spin_rpm = StringVar()
        self.slew_rpm = StringVar()
        self.command_rpm = StringVar()
        self.accel_x = StringVar()
        self.accel_y = StringVar()
        self.accel_z = StringVar()
        self.rate_yaw = StringVar()
        self.rate_pitch = StringVar()
        self.rate_roll = StringVar()
        self.tilt_angle = StringVar()
        self.tilt_heading = StringVar()
        self.rotation_pos = StringVar()
        self.yaw_degrees = StringVar()
        self.g_level = StringVar()
        self.voltage = StringVar()
        self.logfile = StringVar()
        self.recording = StringVar()
        self.timestamp = StringVar()
        self.logfileChoice = IntVar()
        self.recordingChoice = IntVar()
        Style().configure("Bold.TLabel", font=('Lucida Grande', 14, 'bold'))
        self.createWidgets()

    def createWidgets(self):
        Label(self, text="Spin throttle:").grid(row=0, column=0, sticky=E)
        Label(self, text="Slew throttle:").grid(row=1, column=0, sticky=E)
        Label(self, textvariable=self.spin_throttle).grid(row=0, column=1, sticky=W)
        Label(self, textvariable=self.slew_throttle).grid(row=1, column=1, sticky=W)
        Label(self, text="RPM:").grid(row=0, column=2, sticky=E)
        Label(self, text="RPM:").grid(row=1, column=2, sticky=E)
        Label(self, textvariable=self.spin_rpm).grid(row=0, column=3, sticky=W)
        Label(self, textvariable=self.slew_rpm).grid(row=1, column=3, sticky=W)
        Label(self, textvariable=self.command_rpm).grid(row=0, column=4, columnspan=2, sticky=E)
        Label(self, text="Accel (G) x:").grid(row=2, column=0, sticky=E)
        Label(self, text="y:").grid(row=2, column=2, sticky=E)
        Label(self, text="z:").grid(row=2, column=4, sticky=E)
        Label(self, textvariable=self.accel_x).grid(row=2, column=1, sticky=W)
        Label(self, textvariable=self.accel_y).grid(row=2, column=3, sticky=W)
        Label(self, textvariable=self.accel_z).grid(row=2, column=5, sticky=W)
        Label(self, text="Rate (dps) yaw:").grid(row=3, column=0, sticky=E)
        Label(self, text="pitch:").grid(row=3, column=2, sticky=E)
        Label(self, text="roll:").grid(row=3, column=4, sticky=E)
        Label(self, textvariable=self.rate_yaw).grid(row=3, column=1, sticky=W)
        Label(self, textvariable=self.rate_pitch).grid(row=3, column=3, sticky=W)
        Label(self, textvariable=self.rate_roll).grid(row=3, column=5, sticky=W)
        Label(self, text="Tilt angle:").grid(row=4, column=0, sticky=E)
        Label(self, text="heading:").grid(row=4, column=2, sticky=E)
        Label(self, text="Rotation pos:").grid(row=5, column=0, sticky=E)
        Label(self, text="Yaw deg:").grid(row=5, column=2, sticky=E)
        Label(self, textvariable=self.tilt_angle).grid(row=4, column=1, sticky=W)
        Label(self, textvariable=self.tilt_heading).grid(row=4, column=3, sticky=W)
        Label(self, textvariable=self.rotation_pos).grid(row=5, column=1, sticky=W)
        Label(self, textvariable=self.yaw_degrees).grid(row=5, column=3, sticky=W)
        Label(self, text="G level:", style="Bold.TLabel").grid(row=6, column=2, sticky=NE)
        Label(self, textvariable=self.g_level, style="Bold.TLabel").grid(row=6, column=3, sticky=NW)
        Label(self, text="Battery voltage:").grid(row=6, column=0, sticky=NE)
        Label(self, textvariable=self.voltage).grid(row=6, column=1, sticky=NW)
        self.columnconfigure(1, minsize=60)
        self.columnconfigure(3, minsize=60)
        self.columnconfigure(5, minsize=60)
        self.rowconfigure(6, minsize=30)
        Checkbutton(self, variable=self.logfileChoice, command=self.setLogging).grid(
            row=7, column=0, sticky=E)
        Label(self, text="Logfile:").grid(row=7, column=1, sticky=E)
        Label(self, textvariable=self.logfile).grid(
            row=7, column=2, columnspan=4, sticky=W)
        Checkbutton(self, variable=self.recordingChoice, command=self.setRecording).grid(
            row=8, column=0, sticky=E)
        Label(self, text="Data:").grid(row=8, column=1, sticky=E)
        Label(self, textvariable=self.recording).grid(
            row=8, column=2, columnspan=4, sticky=W)
        Label(self, text="Time:").grid(row=9, column=1, sticky=E)
        Label(self, textvariable=self.timestamp).grid(
            row=9, column=2, columnspan=4, sticky=W)
        self.reset = Button(self, text="Reset Arduino", command=self.doReset)
        self.reset.grid(row=10, column=0, columnspan=6, sticky=E+W)

    def setLogging(self):
        if self.master.ser is None or not self.master.ser.isOpen():
            return
        if self.logfileChoice.get():
            self.master.ser.write("#lo;#pv;")
            self.ping_timestamp = datetime.now()
        else:
            self.master.ser.write("#lc;")

    def setRecording(self):
        if self.master.ser is None or not self.master.ser.isOpen():
            return
        if self.recordingChoice.get():
            self.master.ser.write("#d1;")
        else:
            self.master.ser.write("#d0;")

    def doReset(self):
        self.master.ser.write("#z;")

class ParabolaControl(Frame):
    PARABOLA_DURATION = 15000  #ms

    def __init__(self, master=None):
        Frame.__init__(self, master)
        self.parabolaPosition = IntVar()
        self.autostart = IntVar()
        self.autostart.set(1)
        self.running = False
        self.countdownTime = StringVar()
        self.createWidgets()
        self.parabola_number = 1

    def createWidgets(self):
        Label(self, text="Parabola: ").grid(row=0, column=0)
        Checkbutton(self, text="Auto", variable=self.autostart).grid(
            row=0, column=1)
        self.start = Button(self, text="Start", command=self.doStart)
        self.start.grid(row=0, column=2)
        self.bar = Progressbar(self, mode="determinate",
                               maximum=self.PARABOLA_DURATION,
                               variable=self.parabolaPosition)
        self.bar.grid(row=0, column=3, sticky=E+W)
        self.stop = Button(self, text="Cancel", command=self.doStop)
        self.stop.grid(row=0, column=4)
        self.stop.state(['disabled'])
        Label(self, textvariable=self.countdownTime).grid(row=0, column=5)
        self.columnconfigure(3, weight=1)
        self.columnconfigure(5, minsize=100)

    def doStart(self, auto=False):
        self.start.state(['disabled'])
        self.stop.state(['!disabled'])
        self.running = True
        self.startTime = datetime.now()
        self.countdownTime.set("%.1f sec" % (self.PARABOLA_DURATION / 1000.0))
        if self.master.ser is not None and self.master.ser.isOpen():
            self.master.ser.write("#a;")
            if auto:
                self.master.ser.write(
                    "#ndetected parabola start (#%d this invocation);"
                    % self.parabola_number)
                self.parabola_number += 1
            else:
                self.master.ser.write("#nmanually started parabola;")

    def doStop(self, auto=False):
        self.start.state(['!disabled'])
        self.stop.state(['disabled'])
        self.parabolaPosition.set(0)
        self.running = False
        self.startTime = None
        self.countdownTime.set("")
        if self.master.ser is not None and self.master.ser.isOpen():
            if auto:
                self.master.ser.write("#nstopped parabola [time];")
            else:
                self.master.ser.write("#nstopped parabola [manual];")

class App(Frame):
    def __init__(self, master=None):
        Frame.__init__(self, master)
        self.grid(sticky=N+S+E+W)
        self.createWidgets()
        self.setState("disconnected")
        self.ser = None
        self.input_buffer = ""
        self.log_file = None
        self.last_ping = datetime.now()
        self.zero_time = datetime.now()
        self.ping_timestamp = datetime.now()
        self.local_log = open("seedcomm.log", "a")
        self.local_log.write("#START %s;\n" % str(datetime.now()))

    def createWidgets(self):
        top = self.winfo_toplevel()
        top.rowconfigure(0, weight=1)
        top.columnconfigure(0, weight=1)
        self.master_control = MasterControl(self)
        self.spin_control = SpinControl(self)
        self.slew_control = SlewControl(self)
        self.status = Status(self)
        self.master_control.grid(row=0, column=0, columnspan=5,
                                 sticky=E+W, padx=5)
        self.spin_control.grid(row=1, column=0, sticky=N+S+E+W, padx=5)
        self.slew_control.grid(row=1, column=2, sticky=N+S+E+W, padx=5)
        self.status.grid(row=1, column=4, rowspan=2, sticky=N+S+E+W, padx=5)
        self.rowconfigure(0, pad=10)
        #self.rowconfigure(1, weight=1)
        self.columnconfigure(0, weight=1)
        self.columnconfigure(2, weight=1)
        self.columnconfigure(4, weight=1)
        Separator(self, orient=VERTICAL).grid(row=1, column=1, sticky=N+S+E+W)
        Separator(self, orient=VERTICAL).grid(row=1, column=3, rowspan=2,
                                              sticky=N+S+E+W)

        self.go_all = Button(self, text="Go Both (spin + slew)",
                             style="Big.TButton", command=self.doGoAll)
        self.go_all.grid(row=2, column=0, columnspan=3, sticky=E+W, padx=5)

        Separator(self, orient=HORIZONTAL).grid(row=3, column=0, sticky=N+E+W,
                                                columnspan=5, pady=5)
        self.rowconfigure(3, minsize=20)
        note_frame = Frame(self)
        note_frame.grid(row=4, column=0, columnspan=5, sticky=E+W)
        Label(note_frame, text="Note:").grid(row=0, column=0, sticky=E)
        self.note = StringVar()
        self.note_field = Entry(note_frame, textvariable=self.note)
        self.note_field.grid(row=0, column=1, sticky=E+W)
        self.note_field.bind("<Return>", lambda e: self.sendNote())
        self.send_note = Button(note_frame, text="Record", command=self.sendNote)
        self.send_note.grid(row=0, column=2)
        note_frame.columnconfigure(1, weight=1)
        self.parabola_control = ParabolaControl(self)
        self.parabola_control.grid(row=5, column=0, columnspan=5, sticky=E+W)

        self.bind_all("<space>", lambda e: self.doIdle())
        self.bind_all("<Escape>", lambda e: self.ser.write("#t0=0;#t1=0;#r0;"))

    def sendNote(self):
        self.ser.write("#n" + self.note.get().replace(";", "',") + ";")
        self.note.set("")

    def doGoAll(self):
        self.spin_control.doGo()
        self.slew_control.doGo()

    def setState(self, state):
        self.state = state
        self.master_control.setState(state)
        if state == 'disconnected' or state == 'disabled':
            self.spin_control.go.state(['disabled'])
            self.slew_control.go.state(['disabled'])
            self.go_all.state(['disabled'])
        else:
            self.spin_control.go.state(['!disabled'])
            self.slew_control.go.state(['!disabled'])
            self.go_all.state(['!disabled'])

        if state == 'disconnected':
            self.status.reset.state(['disabled'])
            self.send_note.state(['disabled'])
        else:
            self.status.reset.state(['!disabled'])
            self.send_note.state(['!disabled'])

    def periodicCall(self):
        #return
        if not self.ser.isOpen():
            return

        try:
            while True:
                ret = self.ser.read(100)
                if ret == "":
                    break;
                self.input_buffer += ret
            while ";" in self.input_buffer:
                semi = self.input_buffer.index(";")
                line = self.input_buffer[:semi]
                self.input_buffer = self.input_buffer[(semi+1):]
                if "#" not in line:
                    continue
                hash = line.index("#")
                line = line[hash:]
                if line[-2:] == "--":
                    continue
                self.local_log.write(line + "\n")
                words = line.split()
                if words[0] == "#ERR":
                    tkMessageBox.showerror("Arduino error",
                                           "Arduino reported error: %s" % " ".join(words[1:]))
                elif words[0] == "#LOG":
                    if words[1] == "-":
                        self.log_file = None
                    else:
                        self.log_file = words[1]
                    self.status.logfileChoice.set(self.log_file is not None)
                    self.status.logfile.set("NONE" if self.log_file is None else self.log_file)
                elif words[0] == "#DATA":
                    self.handleData(words[1:])
                elif words[0] == "#OK":
                    print line
                    if words[2] == "gatherdata":
                        if int(words[3]) == 1:
                            self.status.recording.set("gathering and updating")
                            self.status.recordingChoice.set(1)
                        else:
                            self.status.recording.set("IGNORING")
                            self.status.recordingChoice.set(0)
                    elif words[2] == "ping":
                        self.zero_time = (self.ping_timestamp -
                                          timedelta(milliseconds=int(words[1])))
                        print "D1: word was ping"
                        self.ser.write("#T%d %s;" % (
                                int(self.zero_time.strftime("%s")),
                                datetime.now().isoformat(" ")))
                elif words[0] == "#READY":
                    print "D2: word was ready"
                    self.ser.write("#lr;#d1;#r0;#pv;")
                    self.ping_timestamp = datetime.now()
                elif words[0] == "#NOTE" or words[0] == "#TIME":
                    pass
                else:
                    tkMessageBox.showerror("Parse error", "Unrecognized report %s" % words[0])
            now = datetime.now()
            if (now - self.last_ping).total_seconds() > 0.3:
                print "ping"
                self.last_ping = now
                self.ser.write("#p;")
            if self.parabola_control.running:
                parabola_ms = int(
                    (now - self.parabola_control.startTime).total_seconds()
                    * 1000.0)
                self.parabola_control.parabolaPosition.set(parabola_ms)
                self.parabola_control.countdownTime.set(
                    "%.1f sec" % (
                        (self.parabola_control.PARABOLA_DURATION -
                         parabola_ms) / 1000.0))
                if parabola_ms > self.parabola_control.PARABOLA_DURATION:
                    self.parabola_control.doStop(True)
                    self.doIdle()
        except Exception as e:
            traceback.print_exception(*sys.exc_info())
            tkMessageBox.showerror("Python exception", e)
        finally:
            self.master.after(50, self.periodicCall)

    def massage_accel(self, laccel, laccel_avg, raccel, raccel_avg):
        # LAccel (and gyro) is 5" from CG
        # RAccel is 13.5" from CG
        # so LAx = aT + (5 in)*w^2
        #    RAx = aT + (13.5 in)*w^2
        # w^2 = (RAx - aT)/(13.5 in)
        # aT = LAx - (5/13.5)*(RAx - aT)
        # (8.5/13.5)*aT = LAx - (5/13.5)*RAx
        # aT = (13.5/8.5)*LAx - (5/8.5)*RAx = translational
        #   acceleration in x-direction. Likewise for z-direction
        #   with LAz and RAz.
        # For y, most of the time no significant centripetal force ->
        # just average.
        return ((13.5/8.5)*laccel_avg[0] - (5.0/8.5)*raccel_avg[0],
                (laccel_avg[1] + raccel_avg[1])/2.0,
                (13.5/8.5)*laccel_avg[2] - (5.0/8.5)*raccel_avg[2],
                raccel_avg[0])

    def massage_rotation(self, rotation):
        Vec = collections.namedtuple('Vec', ['x', 'y', 'z'])
        Quat = collections.namedtuple('Quat', ['w', 'x', 'y', 'z'])

        # multiply two quaternions together
        def qmul(q1, q2):
            r = Quat(q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z,
                     q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y,
                     q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x,
                     q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w)
            normsq = r.w*r.w + r.x*r.x + r.y*r.y + r.z*r.z
            if abs(normsq - 1.0) > 1e-4:
                n = math.sqrt(normsq)
                r = Quat(r.w/n, r.x/n, r.y/n, r.z/n)
            return r

        # rotate a vector by a quaternion
        def qrotate(q, v):
            qconj = Quat(q.w, -q.x, -q.y, -q.z)
            qvec = Quat(0, v.x, v.y, v.z)
            qres = qmul(q, qmul(qvec, qconj))
            return Vec(qres.x, qres.y, qres.z)

        # create a quaternion for an axis and angle of rotation
        def aa2q(axis, angle):
            sa = math.sin(angle/2.0)
            ca = math.cos(angle/2.0)
            return Quat(ca, axis.x*sa, axis.y*sa, axis.z*sa)

        # return a vector normalized
        def vnorm(v):
            norm = math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
            return Vec(v.x/norm, v.y/norm, v.z/norm)

        def vdot(u, v):
            return u.x*v.x + u.y*v.y + u.z*v.z

        qr = Quat(*rotation)
        # find the forward and up vectors for the rotated body
        forward = qrotate(qr, Vec(1., 0., 0.))
        up = qrotate(qr, Vec(0., 0., -1.))
        if up.x == 0 and up.y == 0:
            tilt = 0
            heading = 0
            rpos = math.atan2(forward.y, forward.x)
        else:
            # find projection of up onto xy plane
            up_proj = vnorm(Vec(up.x, up.y, 0))
            # find angle between up and xy plane
            dot = vdot(up, up_proj)
            if dot < -1: dot = -1
            if dot > 1: dot = 1
            theta = math.acos(dot)
            # find angle between up and -z axis
            if up.z < 0:
                tilt = math.pi/2.0 - theta
            else:
                tilt = math.pi/2.0 + theta
            # find angle between up_proj and x axis
            heading = math.atan2(up_proj.y, up_proj.x)
            if heading < 0:
                heading += 2*math.pi  # clamp to [0, 2pi)
            # take actual forward vector, unheading and
            # untilt to find rotation pos
            rpv = qrotate(aa2q(Vec(0, 0, 1), -heading), forward)
            rpv = qrotate(aa2q(Vec(0, 1, 0), tilt), rpv)
            rpos = math.atan2(rpv.y, rpv.x)
            if rpos < 0:
                rpos += 2*math.pi

        r2d = 180.0/math.pi
        return (tilt*r2d, heading*r2d, rpos*r2d)

    def handleData(self, words):
        types = (int, int,
                 float,float,float, float,float,float,float, float,float,float,
                 float,float,float, float,float,float,
                 float,float,float, float,float,float,
                 float,
                 int, float,int, float,int, float,float)
        params = [types[i](words[i]) for i in xrange(len(words))]
        (start_millis, end_millis) = params[0:2]
        gyro = params[2:5]
        rotation = params[5:9]
        angle_sums = params[9:12]
        laccel = params[12:15]
        laccel_avg = params[15:18]
        raccel = params[18:21]
        raccel_avg = params[21:24]
        (voltage, relay_state,
         main_throttle, main_rpm, slew_throttle, slew_rpm,
         commanded_rpm, commanded_accel) = params[24:]
        
        if self.state == "disabled" and relay_state == 1:
            self.setState("armed")
        elif self.state == "armed" and relay_state == 0:
            self.setState("disabled")

        (accel_x, accel_y, accel_z, g_level) = self.massage_accel(
            laccel, laccel_avg, raccel, raccel_avg)
        (tilt_angle, tilt_heading, rotation_pos) = self.massage_rotation(rotation)

        if (math.sqrt(laccel_avg[0]**2 + laccel_avg[1]**2 + laccel_avg[2]**2) < 0.2 and
            self.parabola_control.autostart.get() and
            not self.parabola_control.running and
            start_millis > 1000 and
            (datetime.now() - self.ping_timestamp).total_seconds() > 1.0):
            self.parabola_control.doStart(True)

        st = self.status
        st.spin_throttle.set("%.2f" % main_throttle)
        st.slew_throttle.set("%.2f" % slew_throttle)
        st.spin_rpm.set("%d" % main_rpm)
        st.slew_rpm.set("%d" % slew_rpm)
        st.command_rpm.set("" if commanded_rpm < 50 else ("(cmd:%.1f)" % commanded_rpm))
        st.accel_x.set("%.3f" % accel_x)
        st.accel_y.set("%.3f" % accel_y)
        st.accel_z.set("%.3f" % accel_z)
        st.rate_yaw.set("%.2f" % gyro[2])
        st.rate_pitch.set("%.2f" % gyro[1])
        st.rate_roll.set("%.2f" % gyro[0])
        st.tilt_angle.set("%.2f" % tilt_angle)
        st.tilt_heading.set("%.1f" % tilt_heading)
        st.rotation_pos.set("%.1f" % rotation_pos)
        st.yaw_degrees.set("%.1f" % -angle_sums[2])
        st.g_level.set("%.3f" % g_level)
        st.voltage.set("%.2f" % voltage)
        st.timestamp.set(str(
                self.zero_time + timedelta(milliseconds=start_millis)))

    def doConnect(self):
        if self.state == "disconnected":
            try:
                self.ser = serial.Serial("/dev/cu.RN42-5419-SPP", 115200, timeout=0)
#                self.ser = serial.Serial("/dev/cu.usbmodem1421", 115200, timeout=0)
                self.ser.write("#lr;#d1;#r0;#pv;")
                self.ping_timestamp = datetime.now()
                self.periodicCall()
                self.setState("disabled")
            except IOError as e:
                tkMessageBox.showerror("Serial port",
                                       "Could not open serial port: %s" % e.strerror)
        else:
            self.ser.write("#lf;#d0;")
            sleep(1)
            self.periodicCall()
            self.ser.close()
            self.setState("disconnected")

    def doArm(self):
        if self.state == "disabled":
            # arm
            self.ser.write("#r1;")
            #sleep(5)
            #self.ser.write("#t0=0.100")

        else:
            # kill
            self.ser.write("#t0=0.100")
            self.doIdle()
            self.ser.write("#r0;")

    def doIdle(self):
        # idle
        self.ser.write("#t0=0;#t1=0;")

app = App()
app.master.title('Artificial Gravity Controller')
app.mainloop()
