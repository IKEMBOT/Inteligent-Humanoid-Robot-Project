import numpy as np

from ikpy.chain import Chain
from ikpy.utils import plot
import matplotlib.pyplot as plt
from leg_controller import Leg

port = '/dev/ttyUSB0'

np.set_printoptions(precision=3, suppress=True)

chain = Chain.from_urdf_file("Leg.urdf", 
                             active_links_mask=[False, True, True, True, True, True])

# dxl = Leg(port=port)

target = [-0.0, 0.08, 0.125]
orientation = [0, 0.0, 0]
frame_target = np.eye(4)
frame_target[:3, 3] = target
joints = [0] * len(chain.links)
ik = chain.inverse_kinematics(target, initial_position=joints, target_orientation=orientation, orientation_mode=None)

fk = chain.forward_kinematics(ik)

ik = np.delete(ik, 0)
# dxl.set_angle(ik, radian=True)

print("FK")
print(fk[:3,3])

print("IK")
print(ik)

print("servo")
# print(dxl.get_angle(radian=True))

fig, ax = plot.init_3d_figure()
chain.plot(joints, ax)
# plt.savefig("chain1.png")
plt.show()

