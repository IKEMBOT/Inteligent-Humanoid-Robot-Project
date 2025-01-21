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
To detect the target (ball), I implemented a color space and contour-based shape detection algorithm. Figure 1.3 below shows the detected ball using this vision-based system.  

#### Figure 1.3: Contour-Based Ball Detection
![contour](https://github.com/user-attachments/assets/ff95699c-2a78-4601-b3da-2e27df422240)

The middle of the frame acts as the **setpoint**, and the ball's coordinates are used to calculate the **error** by subtracting them from the setpoint. This error is passed to the PD controller, which computes the control output using the equation below:

$$
u(t) = K_p e(t) + K_d \frac{de(t)}{dt}
$$

Here:
- \( K_p \): Proportional gain, which determines the response to the current error.  
- \( e(t) \): The error at the current time \( t \).  
- \( K_d \): Derivative gain, which determines the response to the rate of change of the error.  
- \( \frac{de(t)}{dt} \): The rate of change of the error.

The calculated control output is then converted into radians to control the servo motors, allowing the robot to adjust its movement and maintain its trajectory toward the ball.

