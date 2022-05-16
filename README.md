# leo-user-image
A Docker image that will build into a linux-based container for the user to test programs for LeoRover

# Build
Build the image using the following command:
```
docker build --network=host -t user_image:1.0 .
```

# Run
Run a container with this image by using the command:
```
docker run -p 9000:80 -it --name user user_container:1.0
```
Then, open your browser and go to the following address:
```
localhost:9000
```

# Make
To build the simulation suite correctly, change into the ```~/catkin_ws``` directory. From there run the following command:
```
catkin_make
```
You should receive an error. Simply run the command a second time, and everything should build properly. Your simulation suite is now all set up.
