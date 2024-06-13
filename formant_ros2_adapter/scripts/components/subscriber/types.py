from std_msgs.msg import (
    Bool,
    Char,
    String,
    Float32,
    Float64,
    Int8,
    Int16,
    Int32,
    Int64,
    UInt8,
    UInt16,
    UInt32,
    UInt64,
)
from sensor_msgs.msg import (
    BatteryState,
    LaserScan,
    NavSatFix,
    PointCloud2,
)

STRING_TYPES = [str, String, Char]
BOOL_TYPES = [Bool, bool]
NUMERIC_TYPES = [
    int,
    float,
    Float32,
    Float64,
    Int8,
    Int16,
    Int32,
    Int64,
    UInt8,
    UInt16,
    UInt32,
    UInt64,
]

OTHER_DATA_TYPES = [NavSatFix, BatteryState, LaserScan, PointCloud2]
