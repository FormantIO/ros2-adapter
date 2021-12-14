## ROS2 adapter functionality

For a full list of Formant telemetry types, see: https://formant.readme.io/docs/how-telemetry-streams-work

#### Basic datapoints
[x] Numeric (UInt, Int, and Float types)
[x] Text (String, Char)
[x] Bitset (Bool)
[x] Location (NavSatFix)
[x] Battery (Battery)

All other input types will be ingested as JSON.

#### Input from multiple fields
[x] Bitset (multiple Bool inputs)
[x] Numericset (multiple UInt, Int, or Float inputs)

#### Rich datapoints
[x] Point Clouds (PointCloud2, LaserScan)
[x] Images and Video (CompressedImage)
[] Localization
[] Transform Tree

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
