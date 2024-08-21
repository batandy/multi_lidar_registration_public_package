# multi_lidar_registration_public_package
<p align = "center">
<img src= "https://github.com/batandy/multi_lidar_registration_public_package/blob/main/docs/aligned_four_lidars.png" alt="aligned four lidars" width="450" height="470">
</p> 

## Descriptions
This project is a ROS2-Humble-based application for processing and registering LiDAR point cloud data from multiple sensors. It includes functionalities for filtering, transforming, and synchronizing point clouds from various LiDAR models like "LIVOX MID360", "LIVOX AVIA" and "Velodyne 16". The processed point clouds are used to extract keypoints and align them for further analysis or usage in robotics applications.

## Table of Contents
- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Configuration](#configuration)
- [Pre-requisites](#pre-requisites)
- [Contributing](#contributing)
- [License](#license)

## Features
- **Multi-sensor Support**: Supports 16-channel spinning LiDARs and solid/dome-type LiDARs.
- **Real-time Point Cloud Synchronization**: Synchronizes source and target point clouds for accurate registration.
- **Keypoint Extraction**: Extracts keypoints from the point clouds for further processing.
- **Field-of-View (FOV) Filtering**: Filters point clouds based on the specified field of view.
- **Interactive FOV Adjustment**: Allows real-time adjustment of the filtering parameters via keyboard input.
- **Visualization**: Publishes markers for visualization in RViz to represent the filtering regions.

## Installation
### Prerequisites
- **ROS2 Humble**
- **PCL (Point Cloud Library)**
- **Eigen Library**
- **YAML-CPP Library**

### Build Instructions
1. Clone this repository:
    ```bash
    git clone https://github.com/batandy/multi_lidar_registration_public_package.git
    cd multi_lidar_registration_public_package
    ```
2. Source the setup script:
    ```bash
    source install/setup.bash
    ```

## Usage

### Supported LiDAR Types
Currently, this package supports the following types of LiDAR:
- 16-channel spinning LiDARs (e.g., Velodyne 16-channel)
- Solid or dome-type LiDARs (e.g., LIVOX MID360, LIVOX AVIA)

### Step 1: Configuration

Before running the nodes, you need to configure the `registration_config.yaml` file located in the `install/share/multi_lidar_registration/config` directory.

Here’s an example configuration:

```yaml
point_cloud_registration_node:
  #mid360 -> mid_callback
  #avia -> avia_callback
  #velodyne -> vlp_callback

  ros__parameters:
    source_topic: "LITE_points"
    source_callback: "vlp_callback"
    target_topic: "mid_points_0"
    target_callback: "mid_callback"
    opposite: false
```

- **source_topic**: Name of the source LiDAR topic.
- **source_callback**: The corresponding callback function for the source LiDAR.
- **target_topic**: Name of the target LiDAR topic.
- **target_callback**: The corresponding callback function for the target LiDAR.
- **opposite**: Set to true if the registered LiDAR needs to be mirrored horizontally.

After modifying the configuration, run the following command to start the keypoint extraction process:

```bash
ros2 launch multi_lidar_registration registration.launch.py
```

This will extract the keypoints and store them in the `install/share/multi_lidar_registration/keypoints/` directory.

### Step 2: Registration

Once the registration node has completed and automatically shut down, modify the `publisher_config.yaml` file located in the `install/share/multi_lidar_registration/config` directory. Below is an example:

```yaml
registration_publisher_node:
  ros__parameters:
    overlap_filtering: false
    use_GICP_: false
    publish_merged_topic: false
    calculate_RT: true
```

- **overlap_filtering**: Enable or disable filtering of overlapping regions.
- **use_GICP_**: Enable or disable precise alignment after initial registration. Even if set to false, you can trigger it later by pressing the 'g' key.
- **publish_merged_topic**: If true, publishes the combined LiDAR point clouds to a single topic.
- **calculate_RT**: If true, calculates the transformation matrix using keypoints; if false, uses the pre-saved transformation matrix.

To start the registration process, run:

```bash
ros2 run multi_lidar_registration publisher
```

The registered point clouds will be published to the following topics:

- If `publish_merged_topic=true`: `merged_transformed`, `merged_baseline`
- If `publish_merged_topic=false`: `<source_topic>_transformed`, `<source_topic>_baseline`, `<target_topic>_baseline`

Where `<source_topic>` is the name of the LiDAR topic you configured earlier. The baseline topics represent the midpoint and horizontal alignment between the two LiDARs.

### Overlap Filtering

To filter the overlapping regions, run:

```bash
ros2 run multi_lidar_registration overlap_filtering
```

You can adjust the filtering settings in real-time using the following keyboard controls:

- `c`, `z`: Adjust minimum angle offset
- `a`, `d`: Adjust maximum angle offset
- `q`, `e`: Rotate the FOV
- `Up`, `Down`, `Left`, `Right`: Move the FOV start point
- `n`: Add a new FOV range
- `m`: Switch to the next FOV range
- `s`: Save current FOV settings to file

Filtered and transformed point clouds will be published to the following topics:

- `filtered_topic`
- `transformed_topic`

Markers representing the filtering regions will be published to:

- `min_filtering_marker`
- `max_filtering_marker`
- `start_filtering_marker`

### Configuration

- **`registration_config.yaml`**: Contains parameters related to point cloud registration, such as source and target topics, filtering options, and keypoint extraction settings.
- **`publisher_config.yaml`**: Defines parameters for publishing the processed point clouds.

### Pre-requisites

For effective point cloud registration, a physical calibration target is required. The target should have the following specifications:

- **Top Section**: A circular disk with a diameter of 30 cm.
- **Bottom Section**: A frustum-shaped lava cone with a height of 84 cm, a top radius of 17 cm, and a bottom radius of 7 cm.
- **Reflective Film**: The reflective film should be applied only to the circular top section and the frustum-shaped bottom section.
