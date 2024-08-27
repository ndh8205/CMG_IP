## Intro

Trajectory tracking allows the system to follow a preset path over time. Using this method, the results of movement can be predicted through simulation without physical experimentation.
Trajectory tracking involves path planning and orbit generation. Path planning is about finding the optimal path to reach a goal from an initial state, given information about the robot and its surroundings. Trajectory generation specifically defines position, velocity, and acceleration functions over time to follow the planned path after path planning is complete.
There are several ways to track trajectories, but for this page we will use PID and FLC.

## Trajectory tracking

Tracking a trajectory requires two states: heading angle and velocity.

If you know the two states mentioned above, tracking the trajectory is simple. For example, if you are at point A and plan to travel to point B in 30 minutes, what information do you need? Those are direction and speed. Using these two information, you can predict where your next location will be every time you move. Let's apply this method to a robot. If you want to know the path the robot moves. We need the error between target position and current position. This is because the error between target position and current position contains two information: heading angle and speed.


<img src="Image/Trajectory_tracking/Trajectory_tracking.png" alt="RCTVC" style="width: 60%;" />

**Figure 1:** Trajectory tracking

As shown in Figure 1, we can define direction d as:

- $d = \sqrt{(x_d - x)^2 + (y_d - y)^2}$

Heading angle $\theta$ as:

- $\theta = \tan^{-1} \left(\frac{y_d - y}{x_d - x}\right)$

We can define the errer as:

- $x_e = x_d - x$

## Simulation

<img src="Image/Trajectory_tracking/Trajectory_tracking_PID.png" alt="RCTVC" style="width: 100%;" />

**Figure 2:** Trajectory tracking using PID
