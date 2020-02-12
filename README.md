# Path Planning Project

## Overview

### Goal

- The goal of this project is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. （尽可能频繁地满足50MPH ～= 25m/s。）

- **Localization**、**Sensor Fusion data**、**A Sparse map list of waypoints** around the highway are provided.

- The car should be able to make one complete loop around the 6946m highway which takes a little over 5 mins. （至少完成一圈耗时略多于5分钟的6946米的行驶。）

- The car should avoid hitting other cars.（碰撞避免。）

- Acceleration <= 10 m/s^2（每2秒计算1次）

  Jerk <= 10 m/s^3（每1秒计算1次）

  AccN（转向时的向心加速度：转向越快 AccN越高）

### Details

- The vehicle moves to a new waypoint every 20ms（每20ms/0.02s 移动至新的点）**cycle = 20ms**
- The vehicle moves 50 frames/times per second（每秒移动50次）

- #### Highway Map（公路图）

  - `data/highway_map.csv`
  - Each waypoint in the list contains `[x,y,s,dx,dy]` values.（每个航路点都有一个（x，y）全局地图位置，以及一个Frenet s值和Frenet d单位法向矢量（分为x分量dx和y分量dy）。）
  - s值是沿道路方向的距离。第一个航点的s值为0，因为它是起点。distance along the road, goes from 0 to 6945.554.
  - Lane width = 4m. 高速公路共有6条车道，每个方向3条车道。 每个车道为4 m宽，并且汽车只能位于右侧3个车道之一中。 除非更改车道，否则汽车应始终在车道内。

- #### Main car's Localization Data

  - ["x"] The car's x position in map coordinates
  - ["y"] The car's y position in map coordinates
  - ["s"] The car's s position in frenet coordinates
  - ["d"] The car's d position in frenet coordinates
  - ["yaw"] The car's yaw angle in the map
  - ["speed"] The car's speed in MPH

- #### Previous path data given to the Planner（smooth transition）

  - Return the **previous list** but with processed points removed, can be a nice tool to show how far along the path has processed since last time.
  - ["previous_path_x"] The previous list of x points previously given to the simulator
  - ["previous_path_y"] The previous list of y points previously given to the simulator

- #### Previous path's end s and d values

  - ["end_path_s"] The previous list's last **point**'s frenet s value
  - ["end_path_d"] The previous list's last **point**'s frenet d value

- #### Sensor Fusion Data, a list of all other car's attributes on the same side of the road.

  - ["sensor_fusion"] A **2d vector of cars** and then that car's 

    `[id，x，y，vx，vy，s，d]`（均为全球变量global variables）

    [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates].

  - `vx`，`vy`值可用于预测未来的汽车位置。 例如，假设被跟踪的汽车沿道路行驶，那么其未来的Frenet预测值将是其当前的s值加上其（已转换的）总速度（m / s）乘以未来经过的时间。



## Prerequisites

### Dependencies 

- cmake >= 3.5

- make >= 4.1

- gcc/g++ >= 5.4

- libuv 1.12.0

- uWebSockets

  - The communication between the simulator and the path planner is done using WebSocket. The path planner uses the uWebSockets WebSocket implementation to handle this communication.

  - Run `install-mac.sh` if your device is Mac OS

  - If you install from source, checkout to commit `e94b6e1`, i.e.

    ```shell
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

- Provided simulator

### Usage

1. Clone this repo.

2. Make a build directory: `mkdir build && cd build`

3. Compile: `cmake .. && make`

4. Run: `./path_planning`.

   ```shell
   > cd build
   > ./path_planning
   Listening to port 4567
   ```

5. Open the simulator and Click `select`to monitor the virtual environment.

![simulator](./images/simulator.png)

#### 

## Implementation

### Prediction

1. This part of the code deals with the localization and sensor fusion data. It intents to reason about the environment. In the case, we concern about following three aspects :

- Whether there is a car in front of us blocking the traffic.

  （前方是否有车阻挡交通）

- Whether there is a car to the right of us making a lane change which is not safe.

   （右侧是否有不安全的车辆在进行变道）

- Is there a car to the left of us making a lane change not safe which is not safe.

   （左侧是否有不安全的车辆在进行变道）

  

2. It is solved by calculating the lane each other car lies and the position it will occur at the end of the last planned trajectory. 

   （通过previous path计算每一个车道上的其他车辆的轨迹及其end point。）

3. A car is considered as "dangerous" when its distance to ego car is less than 30 meters ahead or behind us.

   （当汽车在我们前面或后面的距离小于30米时，就被认为是“危险”的。）

### Behavior

1. This part deals with the problem: **"Which behavior should we take if there is a car in front"**

- Change lane

- Speed up

- Slow down

2. `speed_diff` variable is defined for speed changes while generating the trajectory.（`speed_diff` 在生成轨迹时用于速度更改。 ）

3. This approach helps the car to respond the changing situations fast and efficiently. eg. The ego car will decelerate if there is a car ahead with the distance away less than 30m to avoid collision.

   （反应更迅速和高效，例如当前方距离小于30米处有车辆时，自车将会采取减速以避免碰撞。）

### Trajectory

1. This part calculates the trajectory based on the **speed** and **lane** output from the *behavior*, localization and *previous path points*.

   从行为决策，定位，预测中获得速度与车道信息，生成运行轨迹。

2. The last two points of the previous trajectory (or the car position if there are no previous trajectory) are used in conjunction three points at a far distance to initialize the spline calculation. To make the work less complicated to the spline calculation based on those points, the coordinates are transformed (shift and rotation) to local car coordinates .

   首先，将前一条轨迹的最后两个点（如果没有前一条轨迹，则为自车位置）与远处的三个点结合使用以初始化样条曲线spline。 为了使基于这些点的样条线计算工作更简单，将坐标转换（移动和旋转）为汽车坐标。

3. In order to ensure more continuity on the trajectory, the previous trajectory points are copied to the new trajectory. The rest points are calculated by evaluating the spline and transforming the output coordinates to global coordinates.  Note that the change of velocity is determined by the behavior in order to increase/decrease speed on every single trajectory point instead of the entire trajectory.

   为了确保在轨迹上具有更大的连续性，将先前的轨迹点复制到新轨迹上。 通过评估样条曲线来计算剩余点，并转换为全局坐标。 同时，速度的变化由行为决策决定，以便在每个单个轨迹点而不是整个轨迹上增加/减小速度。



## Performance

### The car is able to drive 4 miles without accident

![driving](./images/driving.png)

The simulator ran 4 miles around 6 minutes without any accident.

### The car drives according to the Speed limit.

No speed limit red warning happened.

### Max Acceleration and Jerk are not exceeded.

No acceleration and jerk warning happened.

### The car does not have collisions.

No collisions happened.

### The car is able to change lanes.

The car kept driving on its lane most of the time, but it was also able to change lanes in terms of traffic situations. For example, there is a a car ahead driving slowly, and it is safe to change lanes if the distance away from other cars is large enough (>=30m).

![highway](./images/highway.gif)

## Conclusion

It is possible to drive without difficulty even with fairly dense traffic, and according to the designed cost, when there is a safe path that guarantees a better speed, the vehicle travels as close to max_speed as possible through lane change if there is slow vehicle ahead. 