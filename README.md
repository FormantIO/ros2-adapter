# Formant ROS 2 Adapter

This adapter allows the Formant Agent to bi-directionally connect Formant streams with ROS 2 topics and services, and to set Formant application configuration parameters as ROS 2 parameters.

## Configuration

The adapter can be configured by either editing the `config.json` file in the `formant_ros2_adapter/scripts` directory, or by pasting full adapter configuration into the "blob data" section of the Formant Agent's configuration page.

A full example configuration is provided at `config_example.json`.

The full configuration schema is available [here](formant_ros2_adapter/scripts/config_schema.json).

Adapter configuration is split into five sections:

### Subscribers

The `subscribers` section defines a list of ROS 2 topics that the adapter will subscribe to, and the Formant streams that they will be mapped to.

| Parameter                 | Description                                          |
| ------------------------- | ---------------------------------------------------- |
| `ros2_topic`              | The ROS 2 topic to pull messages from                |
| `ros2_message_type`       | The type of message to accept                        |
| `formant_stream`          | The name of the Formant stream to ingest to          |
| `ros2_message_paths`      | The list of paths configurations to ingest data from |
| `ros2_message_paths/path` | The path within the ROS 2 message                    |
| `ros2_message_paths/tags` | The tag set to ingest data with                      |
| `ros2_qos_profile`        | The Quality of Service profile for the messages      |

#### Example

```json
"ros2_adapter_configuration": {
    "subscribers": [
        {
            "ros2_topic": "/example_topic",
            "ros2_message_type": "example_msgs/msg/ExampleType",
            "formant_stream": "example.stream",
            "ros2_message_paths": [
                {
                    "path": "example_path",
                    "tags": {
                        "example_key": "example_value"
                    }
                }
            ],
            "ros2_qos_profile": "SYSTEM_DEFAULT"
        }
    ]
}
```

### Publishers

The `publishers` section defines a list of ROS 2 topics that the adapter will publish to, and the Formant streams that they will be mapped to.

The Formant application can send a message on a stream from either:

1. A teleoperation control, such as a button, slider, joystick, or image click
2. A command

In either of these cases, the name of the configured Formant control input must match the `formant_stream` parameter in the adapter.

| Parameter           | Description                                         |
| ------------------- | --------------------------------------------------- |
| `formant_stream`    | The name of the Formant stream to publish data from |
| `ros2_topic`        | The name of the ROS 2 topic to publish data to      |
| `ros2_message_type` | The type of ROS 2 topic to publish data to          |
| `ros2_qos_profile`  | The Quality of Service profile for the messages     |

#### Example

```json
"ros2_adapter_configuration": {
    "publishers": [
        {
            "formant_stream": "example.stream",
            "ros2_topic": "/example_topic",
            "ros2_message_type": "example_msgs/msg/ExampleType",
            "ros2_qos_profile": "SYSTEM_DEFAULT"
        }
    ]
}
```

### Service Clients

The `service_clients` section defines a list of ROS 2 services that the adapter will call, and the Formant streams that they will be mapped to.

Formant commands can be mapped to services by setting the `formant_stream` parameter to the name of the configured Formant command.

Services with zero or one parameter can be called using the following logic:

1. If the service has zero parameters, a command with any parameter (or no parameters) will call it
2. If the service has one numeric parameter, a command with a single numeric parameter will call it
3. If the service has one string parameter, a command with a single string parameter will call it
4. If the service has one boolean parameter:
   1. If the command has no parameters, it will call the service with "true"
   2. If the command has a parameter that maps to a boolean value, it will call the command with that value
      1. True parameters include `["true", "True", "TRUE", "t", "T", "1"]`
      2. False parameters include `["false", "False", "FALSE", "f", "F", "0"]`

| Parameter           | Description                                            |
| ------------------- | ------------------------------------------------------ |
| `formant_stream`    | The name of the Formant stream to accept commands from |
| `ros2_service`      | The name of the ROS 2 service to call                  |
| `ros2_service_type` | The type of ROS 2 service to call                      |

#### Example

```json
"ros2_adapter_configuration": {
    "service_clients": [
        {
            "formant_stream": "example.stream",
            "ros2_service": "/example_service",
            "ros2_service_type": "example_services/srv/ExampleService",
        }
    ]
}
```

### Localization

The Formant localization datapoint includes many fields which are all aggregated to create a single aligned world perspective.

The `localization` section defines a list of ROS 2 topics that the adapter will subscribe to, and the Formant localization datapoint fields that they will be mapped to.

This datapoint uses a special localization manager to aggregate the data from all of the configured topics. It is built to publish only the necessary data in order to save bandwidth.

This configuration section also maps incoming navigation controls such as waypoints from the localization UI to ROS 2 topics.

| Parameter                            | Description                                                        |
| ------------------------------------ | ------------------------------------------------------------------ |
| `formant_stream`                     | The name of the Formant stream to ingest to                        |
| `base_reference_frame`               | The base reference frame to use                                    |
| `odometry_subscriber_ros2_topic`     | The odometry ROS 2 topic name                                      |
| `map_subscriber_ros2_topic`          | The map ROS 2 topic name                                           |
| `point_cloud_subscriber_ros2_topics` | A list of ROS 2 topics to ingest point clouds from                 |
| `path_subscriber_ros2_topic`         | The ROS 2 topic to ingest path messages from                       |
| `goal_subscriber_ros2_topic`         | The ROS 2 topic to ingest goal messages from                       |
| `goal_publisher_ros2_topic`          | The ROS 2 topic to publish goal messages from waypoint UI clicks to|
| `cancel_goal_publisher_ros2_topic`   | The ROS 2 topic to publish waypoint cancellation messages to       |

#### Example

```json
{
  "ros2_adapter_configuration": {
    "localization": {
      "formant_stream": "example.localization",
      "base_reference_frame": "map",
      "odometry_subscriber_ros2_topic": "/odom",
      "map_subscriber_ros2_topic": "/map",
      "point_cloud_subscriber_ros2_topics": [
        {"ros2_topic": "/scan"},
        {"ros2_topic": "/stereo/depth/points"}
      ],
      "path_subscriber_ros2_topic": "/plan",
      "goal_subscriber_ros2_topic": "/goal_pose",
      "goal_publisher_ros2_topic": "/goal_pose",
      "cancel_goal_publisher_ros2_topic": "/move_base/cancel"
    }
  }
}
```

### Transform Tree

The Formant transform tree datapoint includes the trasnform tree from `/tf` and `/tf_static` rooted at a specific `base_reference_frame`(e.g. `base_link`)

The `transform_tree` section requires a `base_reference_frame` to root the tree

| Parameter              | Description                                            |
| ---------------------- | ------------------------------------------------------ |
| `base_reference_frame` | The base reference frame to use for the transform tree |

#### Example

```json
{
  "ros2_adapter_configuration": {
    "transform_tree": {
      "base_reference_frame": "base_link"
    }
  }
}
```

### Numeric Sets

| Parameter                            | Description                                                            |
| ------------------------------------ | ---------------------------------------------------------------------- |
| `formant_stream`                     | The name of the Formant stream to ingest a numeric set to              |
| `ros2_subscribers`                   | The list of subscriber configurations to pull numeric data from        |
| `ros2_subscribers/ros2_topic`        | The ROS 2 topic to pull numeric data from                              |
| `ros2_subscribers/ros2_message_path` | The ROS 2 message path to use to select data from messages on the topic|
| `ros2_subscribers/label`             | The text to use for the label of this value in the numeric set         |
| `ros2_subscribers/unit`              | The text to use for the unit of this value in the numeric set          |

#### Example

```json
"ros2_adapter_configuration": {
    "numeric_sets": [
        {
            "formant_stream": "example.numeric_set",
            "ros2_subscribers": [
                {
                    "ros2_topic": "/example_topic",
                    "ros2_message_path": "example_numeric_value_path_1",
                    "label": "example label 1",
                    "unit": "units"
                },
                {
                    "ros2_topic": "/example_topic",
                    "ros2_message_path": "example_numeric_value_path_2",
                    "label": "example label 1",
                    "unit": "units"
                },
            ]
        }
    ]
}
```

## ROS2 adapter functionality

For a full list of Formant telemetry types, see: https://formant.readme.io/docs/how-telemetry-streams-work

For a list of what is currently supported by the Formant ROS 2 Adapter:

### Basic datapoints

- Numeric (UInt, Int, and Float types) :heavy_check_mark:
- Text (String, Char) :heavy_check_mark:
- Bitset (Bool) :heavy_check_mark:
- Location (NavSatFix) :heavy_check_mark:
- Battery (Battery) :heavy_check_mark:

All other input types will be ingested as JSON.

### Input from multiple fields

- Bitset (multiple Bool inputs) :heavy_check_mark:
- Numeric Set (multiple UInt, Int, or Float inputs) :heavy_check_mark:

### Rich datapoints

- Point Clouds (PointCloud2, LaserScan) :heavy_check_mark:
- Compressed Images :heavy_check_mark:
- Raw Images (into video) :heavy_check_mark:
- Video clips :x:
- Localization (Map, Odometry, Path, etc.) :heavy_check_mark:
- Transform Tree (/tf, /tf_static) :heavy_check_mark:

### Type conversions

Topics will automatically be ingested as their corresponding Formant type:

| ROS topic type                               | Formant datapoint type |
| -------------------------------------------- | ---------------------- |
| Bool, message with bool-valued message paths | bitset                 |
| Char, String                                 | text                   |
| Float, Int, Uint                             | numeric                |
| NavSatFix                                    | location               |
| LaserScan, PointCloud2                       | point cloud            |
| CompressedImage                              | image, video           |

Stream name will be automatically configured from the topic if it is not set. (e.g. "/base/cmd_vel" -> "base.cmd_vel") The `"stream"` configuration can be set to change the stream name of ingested datapoints manually.

### Quality of Service profiles

The adapter currently supports selecting from a set of predefined Quality of Service profiles for publishers and subscribers, based on these default profiles: https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html#qos-profiles

The possible configuration options are:

- SYSTEM_DEFAULT

- SENSOR_DATA

- SERVICES_DEFAULT

- PARAMETERS

- PARAMETER_EVENTS

- ACTION_STATUS_DEFAULT (reliable reliability, transient local durability)

- EXAMPLE_CUSTOM (keep all history)

The EXAMPLE_CUSTOM option shows how additional custom profiles can be created in the code of the adapter.

## Running the adapter

The repo can either be zipped and configured as an adapter in Formant with "Exec command" `./start.sh`, or can be run manually.

If you use custom messages, you must update the `start.sh` script to source your workspace.

## As a ROS 2 Package
Choose where you would like to have your workspace if you do not already have one created. If one already exists, skip the first command.
```
mkdir -p colcon_ws/src

cd colcon_ws/src

git clone <URL for this Repo>

cd ../..

source /opt/ros/<Desired Distro>/setup.bash # (this adapter is meant to work with any ROS2 distribution eloquent+)
```
`colcon build` OR `colcon build --packages-select formant_ros2_adapter` if you have other ROS 2 packages in your workspace that you don't want to build concurrently.

NOTE: If you have custom messages that are not a part of the same workspace as this ros2-adapter, then source them at this point.
```
source install/setup.bash

ros2 run formant_ros2_adapter main.py
```
NOTE: This is untested with ROS 2 Humble
NOTE: This is untested with fastdds