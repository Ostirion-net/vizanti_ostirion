export const ROS2_SCHEMAS = [
  {
    name: "std_msgs/msg/Header",
    definitions: [
      { type: "builtin_interfaces/msg/Time", isComplex: true, name: "stamp" },
      { type: "string", name: "frame_id" }
    ]
  },
  {
    name: "builtin_interfaces/msg/Time",
    definitions: [
      { type: "int32", name: "sec" },
      { type: "uint32", name: "nanosec" }
    ]
  },
  {
    name: "geometry_msgs/msg/Point",
    definitions: [
      { type: "float64", name: "x" }, { type: "float64", name: "y" }, { type: "float64", name: "z" }
    ]
  },
  {
    name: "geometry_msgs/msg/Quaternion",
    definitions: [
      { type: "float64", name: "x" }, { type: "float64", name: "y" }, { type: "float64", name: "z" }, { type: "float64", name: "w" }
    ]
  },
  {
    name: "geometry_msgs/msg/Pose",
    definitions: [
      { type: "geometry_msgs/msg/Point", isComplex: true, name: "position" },
      { type: "geometry_msgs/msg/Quaternion", isComplex: true, name: "orientation" }
    ]
  },
  {
    name: "geometry_msgs/msg/PoseStamped",
    definitions: [
      { type: "std_msgs/msg/Header", isComplex: true, name: "header" },
      { type: "geometry_msgs/msg/Pose", isComplex: true, name: "pose" }
    ]
  },
  {
    name: "nav_msgs/msg/MapMetaData",
    definitions: [
      { type: "builtin_interfaces/msg/Time", isComplex: true, name: "map_load_time" },
      { type: "float32", name: "resolution" },
      { type: "uint32", name: "width" },
      { type: "uint32", name: "height" },
      { type: "geometry_msgs/msg/Pose", isComplex: true, name: "origin" }
    ]
  },
  {
    name: "nav_msgs/msg/OccupancyGrid",
    definitions: [
      { type: "std_msgs/msg/Header", isComplex: true, name: "header" },
      { type: "nav_msgs/msg/MapMetaData", isComplex: true, name: "info" },
      { type: "int8", isArray: true, name: "data" }
    ]
  },
  {
    name: "sensor_msgs/msg/LaserScan",
    definitions: [
      { type: "std_msgs/msg/Header", isComplex: true, name: "header" },
      { type: "float32", name: "angle_min" },
      { type: "float32", name: "angle_max" },
      { type: "float32", name: "angle_increment" },
      { type: "float32", name: "time_increment" },
      { type: "float32", name: "scan_time" },
      { type: "float32", name: "range_min" },
      { type: "float32", name: "range_max" },
      { type: "float32", isArray: true, name: "ranges" },
      { type: "float32", isArray: true, name: "intensities" }
    ]
  },
  {
    name: "geometry_msgs/msg/Vector3",
    definitions: [
      { type: "float64", name: "x" },
      { type: "float64", name: "y" },
      { type: "float64", name: "z" }
    ]
  },
  {
    name: "geometry_msgs/msg/Transform",
    definitions: [
      { type: "geometry_msgs/msg/Vector3", isComplex: true, name: "translation" },
      { type: "geometry_msgs/msg/Quaternion", isComplex: true, name: "rotation" }
    ]
  },
  {
    name: "geometry_msgs/msg/TransformStamped",
    definitions: [
      { type: "std_msgs/msg/Header", isComplex: true, name: "header" },
      { type: "string", name: "child_frame_id" },
      { type: "geometry_msgs/msg/Transform", isComplex: true, name: "transform" }
    ]
  },
  {
    name: "tf2_msgs/msg/TFMessage",
    definitions: [
      {
        type: "geometry_msgs/msg/TransformStamped",
        isComplex: true,
        isArray: true,
        name: "transforms"
      }
    ]
  }
];
