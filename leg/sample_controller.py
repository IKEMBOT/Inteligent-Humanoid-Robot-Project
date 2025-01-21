from leg_controller import Leg
from math import radians
port = '/dev/ttyUSB0'
# port = 'COM3'

dxl = Leg(port=port)
# dxl.init()
test = radians(0)
dxl.set_angle([0,0,0,0,0], radian=True)
print(dxl.get_angle(radian=True))

dxl.close()

