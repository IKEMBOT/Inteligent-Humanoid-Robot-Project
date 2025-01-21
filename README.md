# INTELLIGENT HUMANOID ROBOTICS PROJECT

### Assignment 1.0
This assignment involves an open-loop system where a robot is controlled based on keyboard commands.  
The Inverse Kinematics (IK) method is applied to control one leg of the Mini Darwin robot, which has six degrees of freedom (6-DOF).  
Below are images depicting the mechanical structure of the robot (Figure 1.1) and the maze used in the experiment (Figure 1.2).

| **Figure 1.1: Mini Darwin Robot Leg** | **Figure 1.2: Maze Used for Robot Navigation** |
|:--------------------------------------:|:---------------------------------------------:|
| ![leg](https://github.com/user-attachments/assets/7fc8b985-e32f-465d-8ddc-ea9c76b6d44f) | ![maze](https://github.com/user-attachments/assets/814dc8d0-cae9-41fa-9520-87b2540903fd) |

---

### Assignment 1.1
This assignment involves a closed-loop control system utilizing PD (Proportional, Derivative) control, with vision serving as the observation to provide feedback error.  
In this case, I utilized a color space and contour-based shape detection for the ball to obtain its coordinates. Vision is shown in Figure 1.3.  

The middle of the frame acts as the setpoint, and subtracting it from the ball's coordinates yields the error, which is then used to calculate the PD controller.  
The applied PD method uses only the Proportional (P) and Derivative (D) components, as shown in the equation below. The output is subsequently converted into radians to control the servos.

$$
u(t) = K_p e(t) + K_d \frac{de(t)}{dt}
$$
