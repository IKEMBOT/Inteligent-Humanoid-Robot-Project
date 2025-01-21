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
This assignment involves a closed-loop control system utilizing PD (Proportional, Derivative) control, with vision serving as the observation to provide feedback error. To detect the target (ball), I implemented a color space and contour-based shape detection algorithm. Figure 1.3 below shows the detected ball using this vision-based system.  

#### Figure 1.3: Contour-Based Ball Detection
<div align="center">
    <img src="https://github.com/user-attachments/assets/cca1b3d3-7f4f-4f67-888e-a93c2e468e0d" alt="Contour-Based Ball Detection" width="25%">
</div>

The middle of the frame acts as the **setpoint**, and the ball's coordinates are used to calculate the **error** by subtracting them from the setpoint. This error is passed to the PD controller, which computes the control output using the equation below:

$$
u(t) = K_p e(t) + K_d \frac{de(t)}{dt}
$$

Here:  
Here:  
- ![Kp](https://latex.codecogs.com/png.latex?K_p): Proportional gain, which determines the response based on the current error.  
- ![et](https://latex.codecogs.com/png.latex?e(t)): The error at the current time \( t \).  
- ![Kd](https://latex.codecogs.com/png.latex?K_d): Derivative gain, which determines the response based on the rate of change of the error.  
- ![detdt](https://latex.codecogs.com/png.latex?\frac{de(t)}{dt}): The rate of change of the error over time.

The calculated control output is then converted into radians to control the servo motors, allowing the robot to adjust its movement and maintain its trajectory toward the ball.
