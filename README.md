## ROS2 adapter functionality

For a full list of Formant telemetry types, see: https://formant.readme.io/docs/how-telemetry-streams-work

List of what is currently supported by the Formant ROS2 Adapter:

#### Basic datapoints
- Numeric (UInt, Int, and Float types) :heavy_check_mark:
- Text (String, Char) :heavy_check_mark:
- Bitset (Bool) :heavy_check_mark:
- Location (NavSatFix) :heavy_check_mark:
- Battery (Battery) :heavy_check_mark:

All other input types will be ingested as JSON.

#### Input from multiple fields
- Bitset (multiple Bool inputs) :heavy_check_mark:
- Numericset (multiple UInt, Int, or Float inputs) :heavy_check_mark:

#### Rich datapoints
- Point Clouds (PointCloud2, LaserScan) :heavy_check_mark:
- Images and Video (CompressedImage) :heavy_check_mark:
- Localization (Map, Odometry, Path, etc.) TODO 
- Transform Tree (/tf, /tf_static) TODO

### Configuring the ROS2 Adapter

Add a file `config.json` that contains each ROS topic name to ingest as telemetry under the "streams" key. e.g.

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
            "topic": "/base_scan"
        }
    ]
}
```
