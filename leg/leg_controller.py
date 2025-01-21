import PyDynamixel_v2 as pd
from time import sleep
from math import radians, degrees
import numpy as np

PORT = '/dev/ttyUSB0'
BAUDRATE = 1000000

LEG_ID = [15, 13, 11, 9, 7]
OFFSET = [150, 150, 150, 65, 150]
UPPER_LIMIT = [200, 200, 200, 185, 200]
LOWER_LIMIT = [100, 138, 138,  55, 100]

class Leg():
    def __init__(self, dxl_ids=None, port=None, baudrate = None) -> None:
        port = PORT if port is None else port
        baudrate = BAUDRATE if baudrate is None else baudrate
        self.dxl_ids = LEG_ID if dxl_ids is None else dxl_ids
        
        self._serial = pd.DxlComm(port=port, baudrate=baudrate)
        joints_id = []
        for id in range(len(self.dxl_ids)):
            joints_id.append(pd.Joint(self.dxl_ids[id]))
        self._serial.attach_joints(joints_id)
        self._serial.enable_torques()
        # self.init()
        sleep(2)
        pass

    def init(self):
        self._serial.enable_torques()
        leg_dict = {key: value for key, value in zip(self.dxl_ids, OFFSET)}
        self._serial.send_angles({**leg_dict}, radian=False)
    
    def _clip(self, thetas, radian = False):
        if radian:
            clipped_values = [min(max(value, radians(lower)), radians(upper)) for value, lower, upper in zip(thetas, LOWER_LIMIT, UPPER_LIMIT)]
        else:
            clipped_values = [min(max(value, lower), upper) for value, lower, upper in zip(thetas, LOWER_LIMIT, UPPER_LIMIT)]
        return clipped_values
    
    def _offset(sel , thetas, radian = False):
        if radian:
            result = np.add(thetas , [radians(degree) for degree in OFFSET])
        else:
            result = np.add(thetas , OFFSET)
        return result
        
        
    def set_angle(self, thetas, radian=False): 
        offset_theta = self._offset(thetas, radian)
        clip_thetas = self._clip(offset_theta, radian=radian)
        leg_dict = {key: value for key, value in zip(self.dxl_ids, clip_thetas)}
        self._serial.send_angles(leg_dict, radian=radian)
        
    def get_angle(self, radian=False):
        theta = self._serial.get_angles(radian=False)
        thetas=self._dictToList(theta)
        # offset_theta = self._offset(, radian=radian)
        offset_theta = np.subtract(OFFSET, thetas)
        if radian:
            offset_theta = [radians(theta) for theta in offset_theta]
        return offset_theta
    
    def _dictToList(self, dict):
        thetas = []
        for id in self.dxl_ids:
            thetas.append(dict[id])
        return thetas
    
    def disable_torque(self):
        self._serial.disable_torques()

    def close(self):
        self._serial.disable_torques()
        self._serial.release()