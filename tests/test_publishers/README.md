# Test Publisher

1. Set adapter config to `config_test_publishers.json`
2. Configure joystick in Formant teleop with "API" -> "move.turtlebot3"
3. Run adapter
4. Run `ros2 topic echo /cmd_vel1`
5. Teleop the device in Formant and use the joystick

If publishers are working, you will see Twist messages being published.
