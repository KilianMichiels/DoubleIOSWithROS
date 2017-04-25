# DoubleIOSWithROS

## General info
This project contains an iPad application to connect a Double 1 (Double Robotics) to a ROS network using the rosbridge package.
It is currently personalised for a thesis but you can personalise it for your own use.
RBManager and SocketRocket are required.
MainROSDoubleControl is the interface between the Double 1 and the ROS core.

## How to use:

### To run the application:
- Go to the directory with the podfile.
- Run 'pod update' in terminal.
- Open XCode to run the example.

### To connect to ROS:
- Start 'roscore'
- Start 'roslaunch rosbridge_server rosbridge_websocket.launch'
- On the same computer, check your IP with 'hostname -I'
- Insert this in the application while you are on the same network
- Everything should run

### To control the Double with ROS:
- Run 'getDoubleData.py'
- Run 'setControls.py'
