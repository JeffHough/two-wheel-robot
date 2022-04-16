# Joystick Server and Client
Right now, this is a simple echo server, and the client sends a constant message. Eventually,
I want the client to have a GUI end, where some joystick value is grabbed. Then, the client should
send that joystick (r, theta) to the server. Finally, the server will recieve this message, and then
publish the (r, theta) on the /joystick topic over ROS.

## Issues:
- Not really an issue... but I may not need this at all? ROS2 just automagically takes care of multiple
different machines if they are on the same network with the same ROS_DOMAIN_ID???