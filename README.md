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
