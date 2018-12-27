# **Path Planning Project**

The goals / steps of this project are the following:
* Implement path planning in C++
* Safely navigating around a virtual highway with provided simulator


[//]: # (Image References)

[image1]: ./writeup_images/image1.png "Final Result"
[image2]: ./writeup_images/image2.png "Lane Change Strategy"
[image3]: ./writeup_images/image3.png "Lane Change Strategy 2"
[image4]: ./writeup_images/image4.png "spline"
[video1]: ./final_video.mp4 "Final Video"

## Rubric Points
#### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/1020/view) individually and describe how I addressed each point in my implementation.  


### Compiling
#### 1. Your code should compile.
I completed the project under Ubuntu bash on Windows 10. I didn't modify CMakeLists.txt and other configuration files, so follow below to compile the code.  

~~~sh
cd build
cmake ..
make
./path_planning
~~~

Then, launch Term 3 simulator.

### Valid Trajectories
#### 1. The car is able to drive at least 4.32 miles without incident.

#### 2.The car drives according to the speed limit.

#### 3. Max Acceleration and Jerk are not Exceeded.

#### 4. Car does not have collisions.

![image1]  

As seen above, the car successfully drives almost 4.5 miles without any incidents including speed limit, max acceleration and jerk, and collisions. 

#### 5. The car stays in its lane, except for the time between changing lanes.
#### 6. The car is able to change lanes.

You can check the result in the video (`./final_video.mp4`).  

![video1]  


### Reflection
#### Model Document

My path planning model consists of two parts; one is the lane change strategy, the other is to predict path.

#### `1. Lane Change Strategy`

Basically, lane change strategy is as below.
1. The car never changes lane until there is a vehicle in front of it within 30m in the same lane.  
2. If it is the case, check whether it is okay to change lane to the beside one.  

    2-1. Among all the cars on the road, find the nearest cars on both side.
    - nearest car in front of mine on the left(right) lane  
    - nearest car behind mine on the left(right) lane (The reason why I checked the cars behind mine was to prevent collision).  

    2-2. Check validity  
    - (ex) Currently, I am on the lane #1. The distance with the nearest car in fron of me on the left lane(lane #0) is larger than the distance with the car right in front of me(lane #1), and the distance with the nearest car behind me is larger than 10m, it is assumed it's okay to change lane to the left one.

![image2]
~~~cpp
double left_closest_f = 999;
double left_closest_b = -999;
                  
double right_closest_f = 999;
double right_closest_b = -999;

for (int j = 0; j < sensor_fusion.size(); j++) {
...
    // right lane
    if (lane_distance == 1) {
        if ((car_distance > 0) && (car_distance < right_closest_f)) {
        right_closest_f = car_distance;
        }
        else if ((car_distance <= 0) && (car_distance > right_closest_b)) {
        right_closest_b = car_distance;
        }
    } // left lane
    else {
        if ((car_distance > 0) && (car_distance < left_closest_f)) {
        left_closest_f = car_distance;
        }
        else if ((car_distance <= 0) && (car_distance > left_closest_b)) {
        left_closest_b = car_distance;
        }
    }
...
}
~~~  

   2-3. Choose which lane to go  
    - If both left and right lane are not proper to visit, then the car stick to the current lane with lower speed.  
    - If both left and right lane are proper, then choose the one which has bigger distance value.  

![image3]  

~~~cpp
// check validity
if ((lane != 0) && (left_closest_b < -10) && (left_closest_f > (distance+3))) {
    left_valid = true;
}
if ((lane != 2) && (right_closest_b < -10) && (right_closest_f > (distance+3))) {
    right_valid = true;
}

// choose which lane to go
if (left_valid && right_valid) {               
    lane += ((left_closest_f > right_closest_f) ? (-1) : 1);
}
else if (left_valid) {
    lane -= 1;
}
else if (right_valid) {
    lane += 1;
}
~~~

#### `2. Predict Path`

I used spline library for predicting driving path. At first, I pick 5 sparce points([x, y] coordinates) to generate the path. First two points are current position(at time `t`) and previous position(at time `t-1`). If there is no history(at the very beginning of the simulation), predict previous position with the current point.

~~~cpp
// if there is no history
if (previous_size < 2) {
    double prev_x = car_x - cos(car_yaw);
    double prev_y = car_y - sin(car_yaw);

    ptsx.push_back(prev_x);
    ptsy.push_back(prev_y);
    ptsx.push_back(car_x);
    ptsy.push_back(car_y);
}
else {
    ref_x = previous_path_x[previous_size - 1];
    ref_y = previous_path_y[previous_size - 1];

    double ref_prev_x = previous_path_x[previous_size - 2];
    double ref_prev_y = previous_path_y[previous_size - 2];
    ref_yaw = atan2(ref_y - ref_prev_y, ref_x - ref_prev_x);

    ptsx.push_back(ref_prev_x);
    ptsy.push_back(ref_prev_y);
    ptsx.push_back(ref_x);
    ptsy.push_back(ref_y);
}
~~~

Remaining three points are predicted points.  
~~~cpp
// predicted points
vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

ptsx.push_back(next_wp0[0]);
ptsx.push_back(next_wp1[0]);
ptsx.push_back(next_wp2[0]);

ptsy.push_back(next_wp0[1]);
ptsy.push_back(next_wp1[1]);
ptsy.push_back(next_wp2[1]);
~~~  

For the car to move smoothly, connect the 5 points following the spline function. `N` is the number of dots between the two points and it is decided according to the reference velocity.  

![image4]  

~~~cpp
double target_x = 30;
double target_y = s(target_x); 
double target_dist = sqrt((target_x * target_x) + (target_y * target_y));

double x_add_on = 0;
double N = target_dist / (0.02*ref_vel / 2.24); // miles/hour -> meters/s
            
// N * 0.02 * v = distance
for (int i = 1; i <= 50 - previous_size; i++) {
    // (0, 0), 0 degree
    double x_point = x_add_on + target_x / N; 
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;
    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
}
~~~

#### `3. Speed Control`
To avoid speed limit, max acceleration and jerk, I need to control the vehicle speed.


At the very beginning of the lap, the vehicle need to speed up smoothly. So I set 49.7 to be the upper bound for the speed, and the vehicle will speed up as 0.2 MPH at each turn while it's below 49.7.  

If there is a car within 30m, the vehicle needs to slow down and check if it can change the lane. If it is the case, the vehicle slow down as 0.15 MPH at each turn until the car in front of me is farther than 30m. But I don't want the vehicle to slow down too much, so I set the speed of the car in front of me as a reference, and make speed difference not exceed 20 MPH.

~~~cpp
if (close && (car_speed > (speed_lower_bound-20))) {
    ref_vel -= 0.15;
}
else if (ref_vel < 49.7){
    ref_vel += 0.2;
}
~~~