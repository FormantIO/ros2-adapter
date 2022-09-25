# Formant ROS2 Adapter

This adapter allows the Formant Agent to bi-directionally connect Formant streams with ROS2 topics and services, and to set Formant application configuration parameters as ROS2 parameters.

## Configuration

The adapter can be configured by either editing the `config.json` file in the `formant_ros2_adapter/scripts` directory, or by pasting full adapter configuration into the "blob data" section of the Formant Agent's configuration page.

A full example configuration is provided at `config_example.json`.

The full configuration schema is available [here](formant_ros2_adapter/scripts/config_schema.json).

Adapter configuration is split into five sections:

### Subscribers


### Publishers


### Service Clients


### Localization


### Numeric Sets


## ROS2 adapter functionality

For a full list of Formant telemetry types, see: https://formant.readme.io/docs/how-telemetry-streams-work

For a list of what is currently supported by the Formant ROS2 Adapter:

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
- Transform Tree (/tf, /tf_static) :x:

### Type conversions
Topics will automatically be ingested as their corresponding Formant type:

| ROS topic type                               | Formant datapoint type |
|----------------------------------------------|------------------------|
| Bool, message with bool-valued message paths | bitset                 |
| Message with numeric-valued message paths    | numeric set            |
| Char, String                                 | text                   |
| Float, Int, Uint                             | numeric                |
| NavSatFix                                    | location               |
| LaserScan, PointCloud2                       | point cloud            |
| CompressedImage                              | image, video           |

By default, stream name is automatically configured from the topic. (e.g. "/base/cmd_vel" -> "base.cmd_vel") The `"stream"` configuration can be set to change the stream name of ingested datapoints manually.

## Running the adapter

### As an Adapter or with the `start.sh` Script
The repo can either be zipped and configured as an adapter in Formant with "Exec command" `./start.sh`, or can be run manually.

If you use custom messages, you must update the `start.sh` script to source your workspace.