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
- Localization (Map, Odometry, Path, etc.) :x:
- Transform Tree (/tf, /tf_static) :x:

## Configuring the ROS2 Adapter

### Basic configuration

Edit the file `config.json` in the `formant_ros2_adapter/scripts` directory so that it contains each ROS topic name to ingest as telemetry under the "streams" key. e.g.

```
{
    "streams": [
        {
            "topic": "/cmd_vel"
        },
        {
            "topic": "/rgb/image_raw/compressed"
        },
        {
            "topic": "/depth/points"
        },
        {
            "topic": "/base_scan",
            "stream": "scan"
        }
    ]
}
```

The full configuration schema is available [here](scripts/schema.json).

Topics will automatically be ingested as their corresponding Formant type:

| ROS topic type                               | Formant datapoint type |
| -------------------------------------------- | ---------------------- |
| Bool, message with bool-valued message paths | bitset                 |
| Message with numeric-valued message paths    | numeric set            |
| Char, String                                 | text                   |
| Float, Int, Uint                             | numeric                |
| NavSatFix                                    | location               |
| LaserScan, PointCloud2                       | point cloud            |
| CompressedImage                              | image, video           |

By default, stream name is automatically configured from the topic. (e.g. "/base/cmd_vel" -> "base.cmd_vel") The `"stream"` configuration can be set to change the stream name of ingested datapoints manually.

### Message path configuration

`messagePath` and `messagePaths` can be set to ingest specific values, or multiple values.

```
{
    "streams": [
        {
            "topic": "/battery",
            "stream": "Battery Voltage"
            "messagePath": "voltage"
            "rate": 1.0
        },
        {
            "formantType": "numericset",
            "topic": "/battery",
            "stream": "Battery Set"
            "messagePaths": ["voltage", "current", "charge"],
            "units": ["volts", "A", "Ah"]
            "rate": 0.5
        },
        {
            "formantType": "bitset",
            "topic": "/RegionOfInterest",
            "messagePaths": ["do_rectify"]
        },
    ]
}
```

Setting the `"formantType"` to `"numericset"` or `"bitset"` and specifying multiple values in `"messagePaths"` will ingest multiple fields from a given ROS topic into a single, multi-valued datapoint in Formant.

Setting the `"rate"` to a value in `Hz` will allow you to throttle a topic that may be publishing faster than you want it to be ingested on Formant.

## Running the adapter

### As an Adapter or with the `start.sh` Script

The repo can either be zipped and configured as an adapter in Formant with "Exec command" `./start.sh`, or can be run manually.

Be sure to update this part of the `start.sh` script to source the proper ROS2 distribution:

```
source /opt/ros/eloquent/setup.bash  # this adapter is meant to work with any ROS2 distribution eloquent+
# if you use custom messages, source your workspace here
```

### As a ROS2 Package

Choose where you would like to have your workspace if you do not already have one created. If one already exists, skip the first command.

`mkdir -p colcon_ws/src`

`cd colcon_ws/src`

`git clone <URL for this Repo>`

`cd ../..`

`source /opt/ros/<Desired Distro>/setup.bash` (this adapter is meant to work with any ROS2 distribution eloquent+)

`colcon build` OR `colcon build --packages-select formant_ros2_adapter` if you have other ROS2 packages in your workspace that you don't want to build concurrently.

NOTE: If you have custom messages that are not a part of the same workspace as this ros2-adapter, then source them at this point.

`source install/setup.bash`

`ros2 run formant_ros2_adapter main.py`

- NOTE: This is untested with ROS2 Humble
- NOTE: This is untested with fastdds
- Note this branch uses an experimental version of the formant pip package.
  `python3 -m pip install --force-reinstall --upgrade formant-1.93.20_add_localizaiton_python_3814ae97-py2.py3-none-any.whl`
