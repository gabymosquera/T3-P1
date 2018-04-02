# Term 3 - Path Planning Project Reflexion

[//]: # (Image References)
[video1]: ./video/PathPlanningVideo.mp4
   
### General Information

Please refer to the README document in this repository for more information on how to build, compile, and run this project.


### Project Goals

##### Extracted from the project's rubric:

* The car is able to drive at least 4.32 miles without incident.
	* Incident: exceeding acceleration/jerk/speed, collision, and driving outside of the lanes.
* The car drives according to the speed limit (50 MPH).
* The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.
* The car must not come into contact with any of the other cars on the road.
* The car stays in its lane, except for the time between changing lanes.
* The car is able to change lanes


#### Path Generation Using Splines

By watching the Project Walkthrough Video it became evident that starting with the Path Generation portion of the project was the easiest approach for the project. This path generation creation started very simple, using the code shown below:
```
    double dist_inc = 0.4;
    for (int i = 0; i < 50; i++)
    {
      double next_s = car_s + (i+1) * dist_inc;
      double next_d = 6;
      vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
      next_x_vals.push_back(xy[0]);//next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
      next_y_vals.push_back(xy[1]); //next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
    }
```

This code was very rudimentary as it does not smoothens out the path and violated the jerk limit often. So it became clear that some other method was going to be needed to accomplish a smooth path and smooth transitions in between lane changes. As mentioned by the instructors in the video Walkthrough, an easy way to accomplish that is using splines. Please refer to lines 372 to 489 in main.cpp, which first shows the creation of 3 waypoints in x and y coordinates with a separation of 30m. These points are then converted from map coordinates to vehicle coordinates. After this step the spline is created with a simple line:
```
    //Create a spline
  	tk::spline s;
```

the vehicle x and y coordinates were used to set the spline with the following line of code:
```
  	//Set (x, y) points to the spline
  	s.set_points(ptsx, ptsy);
```

After this step we created vectors `next_x_vals` and `next_y_vals` to store the x and y values into to pass to the simulator for the egocar.

Then some math was required to calculate the points that are to be placed along the spline. These points will be visited every .02 seconds by the simulator. The math is shown in this section of code:
```
	double target_x = 30.0;
  	double target_y = s(target_x);
  	double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

  	...

  	double N = (target_dist/(.02* ref_vel/2.24));
	double x_point = x_add_on + (target_x)/N;
	double y_point = s(x_point);
```

This allows us to calculate x coordinate values for the points in the spline and then use the spline library to extract the y value for that x point --> `y_point = s(x_point)`. Finally these points are rotated back to map coordinates and fed to the simulator.


#### Sensor Fusion Data

After completing the path generation portion, it was clear that the car was not equipped with any kind of logic to avoid cars driving in its same lane, change lanes, etc. To be able to detect cars around our egocar the sensor fusion data was used. Refer to main.cpp line 251, and lines 257 to 296.

Here I created some boolean flags to be used later, I extracted all of the information given by our sensor (id, x, y, vx, vy, s, d) and I assigned a lane to each sensor fusion detected car (`sf_car_lane`):
```
	if (d < 4 && d > 0){sf_car_lane = 0;} //left lane
    else if (d < 8 && d > 4){sf_car_lane = 1;} //middle lane
    else if (d < 12 && d > 8){sf_car_lane = 2;} //right lane
```

Also a very important step was to check the s value for the sensed cars after the .02 second timestep and taking into account their speed and the number s path points. Refer to line 296 in main.cpp.

The final step within sensor fusion was to use the boolean flags in the following manner:

* The `car_same` flag will be true only if the detected car was in the same lane as the egocar and if the detected car was within 30 meters of the ego car. The 30 meters parameter lived in the `close` integer from line 264 of main.cpp.
* The `car_left` flag will be true only if the detected car was on the left of the egocar and if it was within +- 30 meters of the current s position of the egocar.
* The `car_right` flag will be true only if the detected car was on the right of the egocar and if it was within +- 30 meters of the current s position of the egocar.

Finally an additional flag was created, the `car_right_far` which will be true only if the detected car was on the right of the egocar and if it was within +- 60 meters of the current s position of the egocar. The use of this particular flag will make more sense as we dive into my Finite State Machine model.


#### The Finite State Machine Model

For this project it made sense to use a finite state machine which by definition is a model that can be set to exactly one state of the finite number of states it has at any given time.

My FSM has three main states (lines 326 to 369 in main.cpp):
1. When a car is detected within 30 meters ahead of the egocar.
2. When there is no car within 30 meters ahead of the egocar, the egocar is driving on the fast lane and also there is no car detected in the middle lane  within +- 60 meters of the current s value of the egocar.
3. When the reference velocity is under 49.5 mph.

Item #1 will detect if there is a car ahead as mentioned before and then it will first check if there is no car to the left of the egocar (by checking the `car_left` flag) and that the egocar is in lanes 1 or 2. If all of that is true, the car will change lanes to the left `lane -= 1;`. This same item will check for cars to the right of the egocar in the same maner (by checking the `car_right` flag) and checking that the egocar is in lanes 0 or 1. If all true the car will change lanes to the right `lane += 1;`. Finally for this state the FSM will check if the egocar is stuck behind a car and also with cars to either side of it. If that's all true it will check the speed of the car in front of it in mph and try to match it giving itself a "safety net" of 2mph under the detected car's speed.

Item #2 focused on returning the car to the center lane when it was driving in the fast lane but had no cars in front of it anymore. This is a law in certain states so I decided to add some additional code to accomplish it. Here is where we use the `car_right_far` flag, to check that there is no car in the middle lane within +- 60 meters of the egocar's current s position. If there was a car within this limits it is a better option to stay in the fast lane and pass such car before returning to the middle lane.  

Finally item #3 just makes sure that our egocar's velocity is never too much lower than the 50 mph speed limit by increasing it by 0.5 mph if it detects that the current speed is under 49.5 mph. 


##### Video 

The following is a video sped up 4x of my egocar going around the track (over 4.32 miles) without incident and accomplishing all of the above mention points without an issue.

[Video](./video/PathPlanningVideo.mp4)


#### Challenges

For the majority of the project I was comparing the velocity of the cars ahead of my egocar in mps with my car's velocity in mph. This caused the car to decrease it's speed way too much and then increase it back to 49.5 and continue in such loop until being able to change lanes. This also prevented my car from being able to match the speed of the car ahead as closely as possible. Once I realized the issue I made sure to compare both speeds in mph, and I also added logic for my car to always go about 2mph under the speed of the car ahead of it as a safety precaution. This seemed to work pretty well.