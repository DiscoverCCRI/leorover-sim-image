# leo-user-image
A Docker image that will build into a linux-based container for the user to test programs for LeoRover

# Build
Build the image using the following command:
```
docker build --network=host -t sim_image:1.0 .
```

# Run
Run a container with this image by using the command:
```
docker run -p 9000:80 -it --name my_ros_container sim_image:1.0
```
Then, open your browser and go to the following address:
```
localhost:9000
```

# Make
To build the simulation suite correctly, change into the ```~/scripts``` directory. From there run the following command:
```
./setup.bash
```
There is a high probability that this will give you an error. Simply run the program until it does not throw any errors. This means that the workspace for ROS is completely setup.

# Run gazebo
To simulate the LeoRover in gazebo, run the following command:
```
roslaunch leo_gazebo leo_gazbeo.launch
```
This should open a new window for the gazebo simulator. If for some reason, this throws an error, close your terminal, reopen it, and try again.

# Run RViz
To get a video of what the rover is seeing, open a new terminal tab and run the following command:
```
roslaunch leo_viz rviz.launch
```
This should open a new new window for RViz. In this window, check the image checkbox, and open the dropdown menu. Here, make sure that the image topic is set to camera/raw, and that the transport hint is set to raw. Then, the image view window should reflect the view of the camera on the rover.

# RoverAPI
To use DiscoverCCRI's RoverAPI in your python scripts, you will need to include import statements for each component. Each module is stored in the rover_api ROS package as a discover_<object_name>.py file. This means that to import the code, include something like this in your python script:
```
from rover_api.discover_camera import Camera
```
This code imports the camera functionality for the rover. When your script is written, simply run it as an executable, and watch the rover do its thing on gazebo and RViz.
