import Sofa
import Sofa.Core
from math import pi

import tkinter as tkinter
import threading
import numpy


class App(threading.Thread):

    def __init__(self, robot, initAngles=[0., 0., 0., 0., 0., 0.]):
        threading.Thread.__init__(self)
        self.daemon = True
        self.start()
        self.robot = robot
        self.angle1Init = initAngles[0]
        self.angle2Init = initAngles[1]
        self.angle3Init = initAngles[2]
        self.angle4Init = initAngles[3]
        self.angle5Init = initAngles[4]
        self.angle6Init = initAngles[5]

    def reset(self):
        self.angle1.set(self.angle1Init)
        self.angle2.set(self.angle2Init)
        self.angle3.set(self.angle3Init)
        self.angle4.set(self.angle4Init)
        self.angle5.set(self.angle5Init)
        self.angle6.set(self.angle6Init)

    def callback(self):
        self.root.quit()

    def run(self):
        self.root = tkinter.Tk()
        self.root.protocol("WM_DELETE_WINDOW", self.callback)

        tkinter.Label(self.root, text="Robot Controller Interface").grid(row=0, columnspan=6)

        self.angle1 = tkinter.DoubleVar()
        self.angle2 = tkinter.DoubleVar()
        self.angle3 = tkinter.DoubleVar()
        self.angle4 = tkinter.DoubleVar()
        self.angle5 = tkinter.DoubleVar()
        self.angle6 = tkinter.DoubleVar()

        tkinter.Scale(self.root, variable=self.angle1, resolution=0.001, length=400,
                      from_=self.robot.getData('minAngles1'), to=self.robot.getData('maxAngles1'),
                      orient=tkinter.VERTICAL).grid(row=1, column=0)
        tkinter.Scale(self.root, variable=self.angle2, resolution=0.001, length=400,
                      from_=self.robot.getData('minAngles2'), to=self.robot.getData('maxAngles2'),
                      orient=tkinter.VERTICAL).grid(row=1, column=1)
        tkinter.Scale(self.root, variable=self.angle3, resolution=0.001, length=400,
                      from_=self.robot.getData('minAngles3'), to=self.robot.getData('maxAngles3'),
                      orient=tkinter.VERTICAL).grid(row=1, column=2)
        tkinter.Scale(self.root, variable=self.angle4, resolution=0.001, length=400,
                      from_=self.robot.getData('minAngles4'), to=self.robot.getData('maxAngles4'),
                      orient=tkinter.VERTICAL).grid(row=1, column=3)
        tkinter.Scale(self.root, variable=self.angle5, resolution=0.001, length=400,
                      from_=self.robot.getData('minAngles5'), to=self.robot.getData('maxAngles5'),
                      orient=tkinter.VERTICAL).grid(row=1, column=4)
        tkinter.Scale(self.root, variable=self.angle6, resolution=0.001, length=400,
                      from_=self.robot.getData('minAngles6'), to=self.robot.getData('maxAngles6'),
                      orient=tkinter.VERTICAL).grid(row=1, column=5)

        self.root.mainloop()


class RobotGUI(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.robot = kwargs["robot"]
        self.app = App(self.robot, kwargs.get("initAngles", [0., 0., 0., 0., 0., 0.]))

        return

    def reset(self):
        self.app.reset()

    def formatAnglesToSendThroughUSB(self, angles):
        # The value we send through USB should be a vector of char, so an integer between 0 and 254 (255 being the header).
        # We'll thus send the angle in degree. Let's try something really simple first.
        # The interface considers the following range of actuation for each joint: [-pi, pi], let's clamp it to [-127, 127] in degree
        # 255 = header (each time we receive 255 we know it's followed by a vector of six angles)
        # 0 = -127 degrees
        # 127 = 0 degree
        # 254 = 127 degrees

        packetOut = [min(254, max(0, int(angle * 180 / pi + 127))) for angle in angles]
        self.robot.packetOut = packetOut

        return

    def onBeginAnimationStep(self, dt):
        angles = [
            self.app.angle1.get(),
            self.app.angle2.get(),
            self.app.angle3.get(),
            self.app.angle4.get(),
            self.app.angle5.get(),
            self.app.angle6.get()
        ]

        angles = numpy.array(angles)
        self.robot.angles = angles.tolist()

        self.formatAnglesToSendThroughUSB(angles)

        return
