# Camera
# How to use this Dockerfile

### ✅ Steps to Run `depthai-ros` and View Live Camera Feed on the Robot PC

After build the Docker image:  
```bash
docker build -t depthai-test .
```

I did the following on the pc side of the robot:

**1. On the robot PC: Allow Docker to access the host's display (X11):**  
```bash
xhost +local:docker
```

**2. Start the container with display and USB access:**  
```bash
docker run -it \
--privileged \
-e DISPLAY=$DISPLAY \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v /dev/bus/usb:/dev/bus/usb \
--name depthai-test \
depthai-test
```

**3. Inside the container: Launch the `depthai-ros` node:**  
```bash
ros2 launch depthai_examples rgb_stereo_node.launch.py
```

**4. In another terminal: Access the container again** you can do:  
- Check the published topics:  
  ```bash
  ros2 topic list
  ```
  or
- View the camera feed using:  
  ```bash
  ros2 run rqt_image_view rqt_image_view
  ```
  Then select `/color/video/image` from the dropdown to see the live video stream.





