# Test Teleop

## Setup

1. Set adapter config to `config_test_publishers.json`
2. Configure image in teleop with: "API" -> "burger.image", enable encoding with 360p quality
3. Configure joystick in teleop with: "API" -> "burger.move"
4. Configure mouse click in teleop with: "API" -> "burger.click", use "burger.image" as video stream to receive clicks from

## Running
1. Run ROS 2 adapter
2. On ROS 2 device, run `ros2 run image_tools cam2image --ros-args -p burger_mode:=true`
3. Teleop the device, use the joystick and click around on the image

You should see the floating burger images, Twist messages being published on the `/cmd_vel` topic, and Point messages being published on the `/mouse_click` topic.
