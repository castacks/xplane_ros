This write-up is intended to provide a more detailed explanation of the working of XPlaneROS.

### The Nodes

1. **xplane_ros_wrapper**  
 **StateReader** is the class which collects data from XPlane using datarefs via XPlaneConnect plugin. It then publish the odometry data over the topic `/xplane/flightmodel/odom`.   
In addition, it also publishes data in the format used by ROSPlane over ```/fixedwing/xplane/state```.  
**ResetSim** is the class which allows us to spawn the aircraft mid-air. This can be helpful for running episodes which might help in training RL/DL algorithms.  

2. **CommandSender** is the class which interfaces with the user data. It collects data over `/xplane/my_control` topic and then sends it over to XPlane. Currently it also subscribes to `/fixedwing/command` (rosflight_msgs.Command) and converts them into XPlane format.  

3. **TrafficSender** is the class which can be used to recreate real world traffic. The TrajAir dataset can be used to recreate real world traffic scenarios by publishing over `/xplane/adsb_traffic`. The message can then be used for simply as information for functions or it can also be visualized in Python and XPlane.  

4. **RosplaneTuner** uses rqt_config to give step commands which can be helpful for tuning the  PID controller.

### Understanding ROSPlane

The main PID control is predent in **controller_base** and **controller_example**. Under **controller_base**, there is a param called `ai_mode`. Set it to 0 if you want to work with the ROSPlane autopilot. If you want to try out your own algorithm, which also provides some kind of commands in terms of speed, pitch, roll, then you can set the param to 1.