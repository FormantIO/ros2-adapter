# Test Quality of Service Settings
### Subscriber
Create a subscriber with "reliable" reliability (e.g. what's in `config_test_qos.json`).

Publish a message with "best effort" reliability:

`ros2 topic pub /my_string std_msgs/msg/String "data: {key: value}" --qos-profile sensor_data`

QoS is working correctly if you get this message:

`[WARN] [<timestamp>] [<ros2cli>]: New subscription discovered on topic '/my_string', requesting incompatible QoS. No messages will be sent to it. Last incompatible policy: RELIABILITY`

### Publisher
Create a publisher with "best effort" reliability (e.g. what's in `config_test_qos.json`).

Echo that topic with "reliable" reliability:

`ros2 topic echo /my_command_for_publisher --qos-reliability reliable`

QoS is working correctly if you get this message:

`[WARN] [<timestamp>] [<ros2cli>]: New publisher discovered on topic '/my_command_for_publisher', offering incompatible QoS. No messages will be received from it. Last incompatible policy: RELIABILITY`
