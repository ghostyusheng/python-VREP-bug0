from keyboard import add_hotkey
from core.const import const

class Control:
    def registerWASD(self):
        add_hotkey("w", self.w)
        add_hotkey("a", self.a)
        add_hotkey("s", self.s)
        add_hotkey("d", self.d)
        add_hotkey("p", self.p)
        print("register WASD ok!")

    def w(self):
        print('forward')
        const.leftMotorSpeed -= 0.1
        const.rightMotorSpeed -= 0.1
    def s(self):
        print('backward')
        const.leftMotorSpeed += 0.1
        const.rightMotorSpeed += 0.1
    def d(self):
        print('turn right')
        const.leftMotorSpeed += 0.1
        const.rightMotorSpeed -= 0.1
    def a(self):
        print('turn left')
        const.leftMotorSpeed -= 0.1
        const.rightMotorSpeed += 0.1

    def p(self):
        const.debug = not const.debug
        const.leftMotorSpeed = const.rightMotorSpeed = 0
        if const.debug:
            const.RUN=1
            print('continue')
        else:
            const.RUN=0
            print('pause')
