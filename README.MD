# xplane_ros
**xplane_ros** is a ROS wrapper for the XPlane-11 flight simulator. The wrapper provides functionality for extracting aircraft data from the simulator and feeding control commands to control the aircraft in the simulator.   
The main advantage here is due to the fact that XPlane is a realistic simulator and thereby provides more realistic dynamics, responses and images.  
  

`xplane_ros` interfaces with XPlane simulator using the [XPlaneConnect](https://github.com/nasa/XPlaneConnect) plugin.

# XPlane-11
Install the XPlane-11 simulator.

# Setting up XPlaneConnect
First we need to install the XPC plugin. Follow the steps give in the [XPlaneConnect](https://github.com/nasa/XPlaneConnect) repository or in their [Getting Started](https://github.com/nasa/XPlaneConnect/wiki/Getting-Started) page.  

## XPlaneConnect Fixes
There is a chance the plugin might not have loaded correctly. If you don't see the plugin under the Plugin Admin while running XPlane, check the `Log.txt` file in the `XPlane-11` directory.  
  
Here are the fixes to a few possible errors :   
`dlerror:/home/<username>/X-Plane 11/Resources/plugins/XPlaneConnect/64/lin.xpl: undefined symbol: _ZN3XPC15MessageHandlers21CamCallback_RunwayCamEP20XPLMCameraPosition_tiPv`

This issue is resolved by adding ```CameraCallbacks.cpp``` in the ```CMakeLists.txt``` in both the ```add_library``` commands.  
  

`dlerror:/home/<username>/X-Plane 11/Resources/plugins/XPlaneConnect/lin.xpl: wrong ELF class: ELFCLASS32`
This is probably an OS compatibility issue with the pre-built binaries.
[#151](https://github.com/nasa/XPlaneConnect/issues/151)  

While building, the compiler could not find `bits/c++config.h` file. To fix that run the following command.    
`sudo apt-get install gcc-multilib g++-multilib`

Go to ```XPlaneConnect/xpcPlugin```  
```mkdir build```  
```cd build```  
```cmake ..```  
```cmake --build .```  
This will create new .xpl files under a folder named ```XPlaneConnect``` in the `build` directory. Use that for XPlane instead of the pre-built binaries

# Setting up xplane_ros  

**Prerequisites**

1. If not already done so, install [ROS](http://wiki.ros.org/ROS/Installation) (Desktop-Full is recommended).

2. If not already done so, create a catkin workspace with [catkin tools](https://catkin-tools.readthedocs.io/en/latest/):
```bash
sudo apt-get install python-catkin-tools
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
```

**Installation**

1. Move to your catkin workspace: 
```bash
cd ~/catkin_ws/src
```  

2. Install system dependencies:  
```bash
sudo apt-get install python-wstool python-catkin-tools
```  


3. Download repo using a SSH key or via HTTPS: 
```bash
git clone git@bitbucket.org:castacks/xplane_ros.git # SSH
git clone https://rohanblueboybaijal2702@bitbucket.org/castacks/xplane_ros.git # HTTPS
```  

4. Source and compile: 
```bash
catkin build
source ../devel/setup.bash
```  


5. **Temporary**  
xplane_ros will also require you to have [rosplane](https://github.com/rohanblueboybaijal/rosplane/tree/xplane/debug) installed in your local machine.   (Use the `xplane/debug` branch).  
Add `source ~/catkin_ws/devel/setup.bash` in your `.bashrc` file.   
Add the similar command for the `rosplane` workspace incase they are not the same.


# Running xplane_ros 
**Pre-requisites**   
1. Open X-Plane 11 (`X-Plane 11/X-Plane_x86_64`).  
2. Select **New Flight**.  
3. In the LOCATION panel, set the location to Butler airport (ID : KBTP).  
4. Click **Start Flight**.  
5. Allow the aircraft to spawn and then press `B` on the keyboard. (This is to turn off the brakes ; TODO - Add this in code during initialization).

The `default.launch` provides the bare-minimum structure in order to interface with XPlane. The `xplane_ros_wrapper` node will provide the odometry data from XPlane and it will listen to Xplane commands from the user application on the topic `/xplane/my_control` and then send them to XPlane.  

```bash
roslaunch xplane_ros default.launch
```  

The `default.launch` file also has a launch command for the rviz visualization. Uncomment that to load a `.perspective` file with a top down visualization of the odometry.


# Running xplane_ros with ROSplane  
```To be changed (Assumes familiarity with rosplane)```  
`StateReader.py` also packs data in the format needed for ROSplane and publishes that. You can run the rosplane controller along with xplane_ros by running :   

```bash
roslaunch rosplane_sim fixedwing.launch
```  


# Tuning rosplane parameters 
We have tuned the parameters for Cessna_172 (the default aircraft in XPlane) but feel free to try it out for other aircrafts yourself.  

```bash
roslaunch xplane_ros default.launch
```  

In a separate terminal, run :
```bash
roslaunch xplane_ros rosplane_tuner.launch
```  

This will also open an `rqt` window with multiple `rqt_plot` plots.

In a third terminal, run : 
```bash
rosrun rqt_reconfigure rqt_reconfigure
```  


Under the `rqt_reconfigure gui` you should see `fixedwing` and `xplane_ros `. Click on both of them.  
The `fixedwing` setup contains the dynamic reconfigure for `rosplane` parameters. The nomenclature should be obvious once you take a glance at the `params` and `cfg` files provided in `rosplane`. Those are the tuning parameters for the PID loops implemented in `rosplane`.    

The `xplane_ros` portion of the gui provides a kind of easy-to-use interface for you to give certain commands and see the response.   
For example, suppose you want to tune the `roll attitude` parameters. Tick the `hold_roll` box and untick the other boxes. This means that the `autopilot_tuner` node will ignore all the other commanded values and will only run the `roll_hold()` function to set the current `phi` to `phi_c`. You can set the value for `phi_c` using the `roll_step` in the `rqt_reconfigure` gui.   
In the `rqt_plot` you will be able to visualise the **commanded roll**, **current roll** and also the **aileron** commands output by the `roll_hold()` function.  

NOTE : The control surfaces correponding to the unticked boxes will be defaulted to what it was just before you unticked them. 

Henceforth, you should be able to experiment with other control loops like `pitch`, `heading`, `throttle`, etc (each corresponding to a function in `controller_base.cpp` and `controller_example.cpp` in `rosplane`).   
Functionality to come soon :)