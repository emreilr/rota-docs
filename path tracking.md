### What is Path Tracking?

Path tracking refers to the process by which a robotic system, such as a drone or autonomous vehicle, follows a predefined trajectory or path. This involves continuously adjusting the vehicle's position and orientation to minimize the deviation from the desired path. Effective path tracking is crucial for applications like agricultural spraying, autonomous driving, and aerial maneuvers, ensuring that the vehicle performs its tasks accurately and efficiently.

### What is a Local Planner in Robotics?

A local planner is a component of a robotic navigation system responsible for generating short-term movement commands based on the robot's current position, the desired path, and any obstacles in the environment. It operates in real-time, adapting to dynamic changes in the environment and ensuring that the robot can navigate safely and efficiently. Local planners often work in conjunction with global planners, which define the overall route to be taken.

### Algorithms Currently Used by the Industry

In the field of robotics and autonomous systems, particularly in applications such as drone navigation and path tracking, several algorithms have gained prominence due to their effectiveness and adaptability. Below are some of the most commonly used algorithms:

#### 1. Pure Pursuit Algorithm

The Pure Pursuit algorithm is a widely adopted method for path tracking in autonomous vehicles and mobile robots. Its fundamental principle revolves around calculating a "look-ahead point" on the desired trajectory and steering the vehicle towards this point.

**Mechanism**: The algorithm operates by determining a point ahead of the vehicle based on a predefined look-ahead distance. This distance can be adjusted based on the vehicle's speed; a longer distance is typically used at higher speeds to ensure smoother turns. The steering angle is then calculated to direct the vehicle towards this look-ahead point, effectively minimizing the lateral error between the vehicle's current position and the desired path.

**Advantages:

Simple and Effective Design:
Pure Pursuit algorithm is mathematically simple and easy to implement. This enables fast decision making in real-time applications.
Since it requires low processing power, it can work with affordable processors that can be used in drones.

Dynamic Target Tracking:
It works effectively when the target is moving. It is especially suitable for tracking moving enemy drones. It can adapt to dynamic conditions and react to sudden maneuvers of the target.

Smooth and Continuous Movement:
Since it constantly updates the path, drone movements are smoother and more fluid. This provides both fuel/energy efficiency and aerodynamic advantages.

Simple Targeting and Escape Scenarios:
It can be easily set up to track enemy drones or escape from danger.
Disadvantages:

Direct Collision Risk:
While the algorithm constantly tracks the target, it may pose a collision risk since it does not predict the target's maneuvers. When the enemy drone makes a sudden and sharp turn, the following drone may not be able to react.

Poor Performance in Complex Maneuvers:
The algorithm may be inadequate when the target is performing complex maneuvers. It may be particularly ineffective at high speeds and in narrow spaces.
If the enemy drone performs sudden deception maneuvers, Pure Pursuit may have difficulty responding.

Dependence on Target Trajectory:
Since it directly tracks the target's trajectory, the algorithm may make incorrect movements when there is an error in the target's information.
It creates a predictable tracking pattern, which can be easily manipulated by the enemy.

Sensitivity to Environmental Factors:
Does not take environmental obstacles into account. This can cause problems in complex or obstacle-filled battlefields.
It is risky to use without an additional collision avoidance system.

Energy Consumption Problems:
Continuous target engagement can increase energy consumption if not optimized. This can shorten operational time, especially in long-duration missions.

**Limitations**: While effective for many scenarios, the Pure Pursuit algorithm may struggle with sharp turns or complex trajectories, as it does not inherently account for dynamic constraints or the vehicle's kinematic model. This can lead to overshooting or oscillations in the path tracking.

#### 2. Proportional-Integral-Derivative (PID) Controller

The PID controller is a classic control algorithm used extensively in various engineering fields, including robotics. It adjusts the control inputs based on the error between the desired path and the actual position of the vehicle.

**Mechanism**: The PID controller operates by calculating three components: proportional, integral, and derivative. The proportional component responds to the current error, the integral component accounts for past errors, and the derivative component predicts future errors based on the rate of change. By combining these three components, the PID controller generates a control signal that aims to minimize the error over time.

**Advantages:

Simplicity and Easy Application:
PID algorithm has a mathematically simple structure. It can be applied quickly and easily in drone systems.It requires minimal processing power, so it can be run on low-power drone processors.

Real-Time Control:
It can respond quickly and is suitable for instant control. This provides an advantage in situations requiring rapid reaction in drone wars.

Precise Control:
Provides precise control to minimize system errors. For example, it can be used for target-close flight, fixed altitude control or stability.

Wide Application Area:
It can be used in many control tasks from drone stabilization to engine speed control, from positioning to target tracking.

Modularity:
Proportional (P), Integral (I) and Derivative (D) components can be used separately or together. It can be adjusted according to the needs of the drone (e.g. PD, PI).

**Disadvantages:

Inadequacy in Nonlinear Systems:
PID control performs well in linear systems, but cannot adequately adapt to complex dynamics in nonlinear systems such as drones.

Requirement for Manual Tuning:
PID gains (Kp, Ki, Kd) must be adjusted correctly. Incorrect settings can lead to unstable system operation or poor performance.
It is difficult to continuously optimize gain settings for different situations, especially in drone wars.

No Prediction Ability:
PID only tries to correct the current error. It cannot predict the future movements of the target. This can be disadvantageous in tracking moving targets.

Sensitivity to Environmental Effects:
It may be inadequate against sudden wind changes or external disturbances. Being sensitive to environmental factors, especially in drone wars, can make stability difficult.

Lack of Performance in Complex Tasks:
More advanced algorithms may be needed in obstacle detection, complex maneuvers and dynamic environments (e.g. nMPC).

**Limitations**: One of the main challenges with PID controllers is the tuning process, which can be time-consuming and may require expert knowledge. Furthermore, PID controllers may not perform well in highly dynamic environments or when dealing with non-linear systems, as they do not account for the vehicle's dynamics or constraints.

#### 3. Model Predictive Control (MPC)

**Nonlinear Model Predictive Control (nMPC)** is a control algorithm designed to handle systems with nonlinear dynamics. Here’s an explanation without using bullet points:

nMPC works by predicting the future behavior of a system over a specified time horizon. It uses a mathematical model of the system to simulate how inputs will affect its state. The algorithm optimizes these inputs to achieve desired goals, such as following a path or avoiding obstacles, while also respecting system constraints like speed limits or physical boundaries.

At every time step, nMPC solves an optimization problem. This problem minimizes a cost function, which often includes terms for tracking accuracy (e.g., how closely the system follows a target path) and control efficiency (e.g., minimizing energy usage). The result is a sequence of control actions, but only the first action is applied. Afterward, the process repeats with updated information, ensuring the control adapts to real-time changes in the environment or system state.

nMPC excels in handling complex, nonlinear systems like drones, where the relationship between inputs (e.g., motor commands) and outputs (e.g., position or speed) is not straightforward. Its ability to predict and optimize over time makes it ideal for dynamic scenarios, such as navigating through obstacles or evading threats. However, nMPC requires significant computational power because it solves a nonlinear optimization problem at each step. It also depends heavily on the accuracy of the system model; an incorrect model can lead to poor performance.

In summary, nMPC is a powerful but computationally intensive algorithm best suited for systems that require high precision and operate in dynamic, complex environments. It is widely used in robotics, autonomous vehicles, and aerospace applications.

**Advantages:**
Effective in Nonlinear Systems:
It is successful in systems with complex and nonlinear dynamics such as drones. It is suitable for situations that require high maneuverability and precise control.

Ability to Address Multiple Constraints:
It can take into account speed, acceleration, direction and energy constraints at the same time. For example, it can work with both target tracking and collision avoidance constraints in a drone war.

Foresight and Planning:
It creates optimal control signals by predicting the future states of the system. This predicts the movements of enemy drones and provides a strategic advantage.
It can even try to predict complex maneuvers or unpredictable movements of the target.

Ability to Work in Dynamic Environments:
It can quickly adapt to environmental changes. For example, if the location of obstacles on the battlefield changes, it can dynamically update the control strategy accordingly.

Energy Efficiency:
It can minimize energy consumption as it produces optimal solutions. This extends the drone operation time and provides an advantage during the war.

Obstacle and Collision Avoidance:
It can plan safe routes by considering the obstacles and other drones in the environment. This is a great advantage in intense battlefields.

**Disadvantages:**
High Computational Cost:
Calculating optimal control signals in nonlinear systems requires a lot of processing power. This can be a limitation in real-time drone applications.
It may not be suitable for low-power drone processors; therefore, more powerful hardware is needed.

Sensitivity to Modeling Errors:
Since it depends on the nonlinear model of the system, modeling errors can negatively affect control performance. An incorrect system model can cause the drone to move incorrectly.

Real-Time Application Challenges:
Solving complex optimization problems can be time-consuming. In drone wars that require high speed and fast response, delays can degrade performance.

Parameter Tuning Challenges:
The prediction horizon (horizon length) and other parameters need to be carefully tuned. Incorrect settings can cause the algorithm to work slowly or unstable.

Constraint Overload:
Since it addresses many constraints simultaneously, it may not be able to find a solution in some cases or may perform excessive calculations. This may cause the drone to exhibit more complex control behavior than expected.

#### 4. Dynamic Window Approach (DWA)

The Dynamic Window Approach (DWA) is a local path planning algorithm that focuses on the robot's velocity and acceleration constraints to generate feasible trajectories in real-time. It is particularly useful for mobile robots navigating in dynamic environments.

**Mechanism**: DWA operates by evaluating a set of possible trajectories based on the robot's current velocity and acceleration limits. It generates a "dynamic window" of feasible velocities and then simulates the resulting trajectories over a short time horizon. The algorithm selects the trajectory that optimizes a predefined cost function, which typically includes terms for distance to the goal, collision avoidance, and smoothness of the trajectory.

**Advantages:

**Adaptation to Dynamic Environments:**
DWA responds quickly to sudden changes in the environment (e.g. movement of enemy drones or obstacles). This is a great advantage in intense battlefields.

**Real-Time Computation:**
The algorithm makes speed and direction choices within a certain time window. This provides a fast and efficient solution for real-time applications.

**Collision Avoidance:**
Minimizes the risk of collision with close-range obstacles and enemy drones. This is especially important in crowded airspace.

**Addressing Kinematic Constraints:**
Makes control decisions by considering the drone's acceleration and turning capacity. This ensures that it does not exceed physical limitations.

**Optimal Target Tracking:**
Chooses the fastest and safest route to the target while planning a safe path between obstacles.

**Simple Implementation:**
Does not require complex modeling. Can work with the drone's current speed and position information.

**Disadvantages:

**Limited Prediction Ability:**
DWA only makes short-term movement planning. It is inadequate to predict the long-term movements of enemy drones.

**Computational Load:**
Continuously calculating the dynamic obstacle and speed field requires high processing power, especially in cases where there are many obstacles.

**Lack of Global Optimization:**
DWA makes local decisions and can sometimes miss a better global route. This can be disadvantageous for long-range targets.

**Fast-Moving Obstacles:**
It can sometimes be inadequate against fast and complex moving targets such as enemy drones.

**Stuck in Complex Environments:**
In cases where there are many obstacles, the algorithm may enter a loop and not be able to proceed (e.g. a narrow corridor or a congested area).

**Coordinated Movement of Target and Obstacle:**
If the target and obstacle move in the same direction, it may be difficult for the algorithm to find a safe path.

**Limitations**: While DWA is effective for local planning, it may not perform well for global path planning, as it primarily focuses on immediate trajectories. Additionally, the quality of the generated paths can be sensitive to the chosen cost function and parameters, requiring careful tuning for optimal performance.

### Which Algorithm is More Suitable for Us?

The selection of the most suitable algorithm for our project depends on several factors, including the specific requirements of the application, the operational environment, and the desired performance characteristics.

### How Can We Use Those Algorithms with Our System?

To implement these algorithms in our drone system, you would typically follow these steps:

1. **Integrate the Algorithm**: Begin by incorporating the chosen algorithm into the drone's flight control software. This involves ensuring that the algorithm can receive real-time data from various sensors, such as GPS, Inertial Measurement Units (IMUs), and other relevant inputs.
    
2. **Define the Path**: Set up a predefined path or trajectory that the drone needs to follow. This path can be generated using a global planner, which takes into account the overall mission objectives and constraints.
    
3. **Real-Time Adjustments**: Ensure that the local planner can make real-time adjustments based on the drone's current position and any detected obstacles. This may involve implementing feedback mechanisms that allow the algorithm to respond to changes in the environment or the drone's state.
    
4. **Testing and Tuning**: Conduct simulations and field tests to fine-tune the algorithm parameters for optimal performance. This step is crucial for identifying any issues and ensuring that the algorithm operates effectively under various conditions.
    
5. **Performance Monitoring**: Implement monitoring systems to evaluate the performance of the algorithm during operation. This can include logging data on tracking accuracy, response times, and other relevant metrics to assess the effectiveness of the chosen algorithm.
    

### How Can We Evaluate the Performance of Those Algorithms So We Can Select the Best One?

To evaluate the performance of the algorithms and select the best one for our application, consider the following metrics:

1. **Tracking Accuracy**: Measure how closely the drone follows the desired path. This can be quantified by calculating the lateral error between the drone's position and the target trajectory over time.
    
2. **Response Time**: Assess how quickly the system can react to changes in the environment or path. This can be measured by analyzing the time taken for the drone to adjust its trajectory in response to new information.
    
3. **Stability**: Evaluate the smoothness of the drone's movements and its ability to maintain control during maneuvers. This can be assessed by examining the control inputs and the resulting trajectory for oscillations or abrupt changes.
    
4. **Robustness**: Test how well the algorithm performs under various conditions, such as different speeds, payloads, and environmental factors. This can involve conducting tests in diverse scenarios to identify any weaknesses or limitations.
    
5. **Simulation and Real-World Testing**: Use both simulation environments and real-world tests to gather data on performance metrics. Simulations can provide a controlled environment for initial testing, while real-world tests can validate the algorithm's performance in practical applications.
    
6. **Comparative Analysis**: Conduct a comparative analysis of the algorithms based on the collected performance metrics. This can help identify the strengths and weaknesses of each algorithm, allowing for an informed decision on the most suitable option for our specific requirements.