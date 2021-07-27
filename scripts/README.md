# Bare Minimum XPlane interface  
**StateReader** is the class which collects data from XPlane using datarefs via XPlaneConnect plugin. It then publish the odometry data over the topic `/xplane/flightmodel/odom`.  

TODO : Instead of publishing odometry from StateReader we could keep a custom message which includes odometry among other things like airspeed, ground speed, etc. Then we could keep a separate node altogether which converts this message into a specific type based on the controller we're using or whatever other application we might wanna develop.

All the datarefs are first collected and then the request is sent. In order to extract more information, all you need to do is `append` :).  
For more information about information :p check out the datarefs in XPlane online.  

---

**CommandSender** is the class which interfaces with the user data. It collects data over `/xplane/my_control` topic and then sends it over to XPlane. Currently it also subscribes to `rosplane actuator messages` and converts them into XPlane format.

TODO/ALT : Keep a separate node for these conversions of commands in different formats. It would make the things more modular.

---

**xplane_ros_wrapper** is the node which actually interfaces with XPlace.   
It instantiates objects of type **StateReader** and **CommandSender**. And viola! The interface is ready!

---

**ned_to_viz.py** is a utility to subscribe to odometry data in NED format and apply a transformation which eases visualization in RVIZ.  
Same is the case with **rosplane_path_viz.py**. 

---

# Tuning rosplane 
Our path following and control architecture is heavily based on rosplane.   
rosplane makes use of `Dubins path` for trajectory generation. It assumes constant altitude and constant velocity while following curves.  

Adding a utility to tune the PID parameters seemed to make the task easier for us and we hope you can also make use of this incase you want to try out different aircrafts or improve the performance of cessna itself.  

## Running rosplane_tuner
First start a new flight.   
Then run   
```bash 
roslaunch xplane_ros default.launch rosplane_tuner:=true
```

This will spawn the aircraft at the same latitude and longitude but at a height. It will also give an initial velocity to the aircraft along with throttle so that it stays reasonably stable and you can then start trying various parameters.  
NOTE : The initial configuration we have provided is solely based on heuristics and tests so that the aircraft doesn't start going haywire right after the spawn. You can play around with those parameters too. Go through the comments in the code to see what is happening where.  

This Python script gives us the utility to tune **ROLL**, **PITCH** and **COURSE** parameters.  
In the `rqt_reconfigure` GUI, open the xplane_tuner and fixedwing_autopilot windows.   
In the xplane_tuner window you should be able to boxed like `hold_roll, hold_pitch, hold_course` and below that there should be a slider to give values for those commands. The convention used is the standard one.  

Use the `roll_step` and `pitch_step` to give a value at which you want the `roll` and `pitch` to be stabilized. For course, change the numbers in `chi_c`. 

Running the first command should also open an `rqt` window which would be in the format of `commanded  value, actual value, actuator value`. Change the parameters of PID to see what response is the best.  

rosplane has additional control loops like `airspeed_with_pitch_hold` etc. For those we thought it might be best to make the aircraft follow a pattern and tune everything together. 

Run
```bash
bash utils/pattern_following.sh 
```
as mentioned in the main README.

Again, you should get a bigger `rqt` window with similar format. The waypoints commanded form a sqaure and should give you a decent chance to observe the performace of the PID controller.  

Another thing to consider tuning are the vector field parameters. In order to follow a line or an orbit, the aircraft must change the heading to be on the correct path. Those commanded values for chi_c are generated via a vector field formula.