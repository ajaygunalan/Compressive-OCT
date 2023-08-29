
**Warning**

Never begin by connecting the CALM USB, instead start everything else first. Always initiate CALM and check its operation with a pen before connecting the USB. If you don't follow this order, CALM might vibrate. For further instructions, refer to the CALM.pdf

**Update**
ROS has updated the `rostopic` command. Thus, the new synatx is:
``rostopic pub /ralp_msgs/teensy_input ralp_msgs/teensy_input '{buttons: 1, deltax: 0.0, deltay: -0.5}'
publishing and latching message. Press ctrl-C to terminate
``
