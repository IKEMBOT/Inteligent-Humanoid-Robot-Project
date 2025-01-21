import numpy as np
from ikpy.chain import Chain
from ikpy.utils import plot
import matplotlib.pyplot as plt
import pygame
import sys
from leg_controller import Leg

# Initialize Pygame
pygame.init()
# Set up Pygame display
width, height = 50, 100
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Robot Leg Control")

# Define the port for your hardware communication
port = '/dev/ttyUSB0'
dxl = Leg(port=port)
# Set numpy printing options
np.set_printoptions(precision=3, suppress=True)

# Load the URDF file to define the robot chain
chain = Chain.from_urdf_file("Leg.urdf", active_links_mask=[False, True, True, True, True, True])

target_position = [-0.0, 0.00, 0.131]
orientation = [0, 0.0, 0]

frame_target = np.eye(4)
frame_target[:3, 3] = target_position

joints = [0] * len(chain.links)

# Main loop
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_UP:
                target_position[1] += 0.01
            elif event.key == pygame.K_DOWN:
                target_position[1] -= 0.01
            elif event.key == pygame.K_RIGHT:
                target_position[0] -= 0.01
            elif event.key == pygame.K_LEFT:
                target_position[0] += 0.01
            elif event.key == pygame.K_a:
                target_position[2] += 0.01
            elif event.key == pygame.K_d:
                target_position[2] -= 0.01
            elif event.key == pygame.K_ESCAPE:
                print("EXIT")
                pygame.quit()
                sys.exit()

            ik = chain.inverse_kinematics(target_position, initial_position=joints, target_orientation=orientation, orientation_mode=None)
            fk = chain.forward_kinematics(ik)
            ik = np.delete(ik, 0)
            # Uncomment the following lines if you want to interact with your Leg class
            dxl.set_angle(ik, radian=True)
            print(dxl.get_angle(radian=True))
            # Print results
            print("FK")
            print(fk[:3, 3])
            print("IK")
            print(ik)

            # Create a 3D plot of the robot chain
            fig, ax = plot.init_3d_figure()
            chain.plot(joints, ax)

    # Update the Pygame display
    pygame.display.flip()

    # Control the frame rate (you can adjust the delay accordingly)
    # pygame.time.delay(50)
