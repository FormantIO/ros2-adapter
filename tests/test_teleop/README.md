# Test Teleop

## Setup

1. Set adapter config to `config_test_publishers.json`
2. Configure image in teleop with: "API" -> "burger.image", enable encoding with 360p quality
3. Configure joystick in teleop with: "API" -> "burger.move"
4. Configure a button in teleop with: "API" -> "burger.button"
5. Configure mouse click in teleop with: "API" -> "burger.click", use "burger.image" as video stream to receive clicks from

## Running
1. Run ROS 2 adapter
2. On ROS 2 device, run `ros2 run image_tools cam2image --ros-args -p burger_mode:=true`
3. Teleop the device, use the joystick and click around on the image and on the button

You should see the floating burger images, Twist messages being published to the `/cmd_vel` topic, Point messages being published to the `/mouse_click` topic, and Bool messages being published to the `/teleop_button` topic.
