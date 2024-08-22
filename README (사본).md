
# Multi LiDAR Registration
<p align="center">
<img src="https://github.com/batandy/multi_lidar_registration_public_package/blob/main/docs/aligned_four_lidars.png" alt="aligned four lidars" height="470">
</p>

## Descriptions
This project is a ROS2-Humble-based application for processing and registering LiDAR point cloud data from multiple sensors. It includes functionalities for filtering and transforming point clouds from various LiDAR models like "LIVOX MID360," "LIVOX AVIA," and "Velodyne 16." The processed point clouds are used to extract keypoints and align them for further analysis or usage in robotics applications.

## Table of Contents
- [Installation](#installation)
- [Usage](#usage)
- [Pre-requisites](#pre-requisites)
- [Configuration](#configuration)
- [Registration](#registration)
- [Overlap Filtering](#overlap-filtering)

## Installation
### Prerequisites
- **Ubuntu 22.04**
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

### Pre-requisites

<p align="center">
<img src="https://github.com/batandy/multi_lidar_registration_public_package/blob/main/docs/calibration_target.jpg" alt="Calibration Target" height="470">
</p>

For effective point cloud registration, a physical calibration target is required. The target should have the following specifications:

- **Top Section**: A circular disk with a diameter of 30 cm.
- **Bottom Section**: A frustum-shaped lava cone with a height of 84 cm, a top radius of 17 cm, and a bottom radius of 7 cm.
- **Reflective Film**: The reflective film should be applied only to the circular top section and the frustum-shaped bottom section.

## Usage

### Supported LiDAR Types
This package currently supports the following types of LiDAR:
- 16-channel spinning LiDARs (e.g., Velodyne 16-channel)
- Solid or dome-type LiDARs (e.g., LIVOX MID360, LIVOX AVIA)

### Step 1: Configuration

Before running the nodes, you need to configure the `registration_config.yaml` file located in the `install/share/multi_lidar_registration/config` directory.

Here’s an example configuration:

```yaml
point_cloud_registration_node:
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
- **opposite**: Set to true if the registered LiDAR needs to be mirrored horizontally. This is necessary when two LiDARs are facing opposite directions.

#### Running Keypoints Extraction

After configuring the `registration_config.yaml` file according to your LiDAR setup, start the keypoint extraction process by executing the following command:

```bash
ros2 launch multi_lidar_registration registration.launch.py
```

This process involves several key steps:

1. **Point Cloud Reception**:
   - The system receives point clouds from various LiDAR sensors, including Velodyne (16 channels), MID360, and AVIA.
   - These point clouds are processed using the Point Cloud Library (PCL) to prepare for keypoint extraction.

2. **Keypoint Extraction**:
   - The primary objective is to extract distinctive features from the scanned point cloud data using techniques such as **Passthrough filtering**, **Voxel Grid filtering**, and **Principal Component Analysis (PCA)**.
   - The extracted keypoints are crucial for aligning or registering point clouds from different sensors or different positions of the same sensor.

    <p align = "center">
    <img src= "https://github.com/batandy/multi_lidar_registration_public_package/blob/main/docs/keypoint_ex.png" alt="aligned four lidars" height="470">
    </p> 
   *This is an example of the keypoint extraction process.*

3. **Saving Keypoints**:
   - Keypoints are saved in the `install/share/multi_lidar_registration/keypoints/` directory as `source_keypoint.txt` and `target_keypoint.txt`.
   - These files represent the keypoints of the source and target point clouds, respectively.

4. **Additional Functionalities**:
   - A custom point type structure (`PointXYZIR`) is included to handle the ring structure from Velodyne LiDAR. This structure includes fields for XYZ coordinates, intensity, and ring index.
   - The process also manages multiple LiDARs simultaneously, with flags and counters (`src_finish`, `trg_finish`, `src_cnt`, `trg_cnt`) to monitor the completion status of keypoint extraction.

### Step 2: Registration

After the registration node has completed, modify the `publisher_config.yaml` file located in the `install/share/multi_lidar_registration/config` directory. Below is an example:

```yaml
registration_publisher_node:
  ros__parameters:
    overlap_filtering: false
    use_GICP_: false
    publish_merged_topic: false
    calculate_RT: true
```

- **overlap_filtering**: Enable or disable filtering of overlapping regions.
- **use_GICP_**: Enable or disable precise alignment after initial registration. You can trigger it later by pressing the 'g' key even if it is set to false.
- **publish_merged_topic**: If true, publishes the combined LiDAR point clouds to a single topic.
- **calculate_RT**: If true, calculates the transformation matrix using keypoints; if false, uses the pre-saved transformation matrix.

#### Running the Point Cloud Registration

To start the registration process, execute the following command:

```bash
ros2 run multi_lidar_registration publisher
```

This command initiates the registration process using the extracted keypoints and the configuration settings. Here’s a detailed breakdown of what happens:

1. **Transformation Matrix Calculation**:
   - Using the extracted keypoints, the system calculates a transformation matrix (RT) via Singular Value Decomposition (SVD), providing an initial rough alignment.

2. **Refinement with GICP**:
   - The Generalized Iterative Closest Point (GICP) algorithm refines the initial alignment by minimizing the distance between corresponding points, leading to more accurate registration.

3. **Publishing Registered Point Clouds**:
   - The registered point clouds are published to ROS topics based on the **publish_merged_topic** parameter:
     - **If true**: The point clouds are combined into a single set of topics (`merged_transformed`, `merged_baseline`).
     - **If false**: The point clouds are published separately (`<source_topic>_transformed`, `<source_topic>_baseline`, `<target_topic>_baseline`).

4. **Overlap Filtering**:
   - The overlap filtering feature helps manage point clouds by removing specific regions from the target LiDAR's data. This prevents issues such as excessive points in overlapping areas, which could cause errors during further analysis.

5. **Binary File Recording**:
   - If `start_bin_recording` is enabled, the system records the binary data of the processed point clouds for later analysis or debugging.


### Overlap-Filtering

The `overlap_filtering_controller.cpp` file is responsible for performing overlap filtering on LiDAR point clouds. This process allows users to adjust and apply filtering to specific overlapping regions in real-time.

![Overlap Filtering Example](path/to/your/image.png)
*This is an example of the overlap filtering process.*

To filter overlapping regions, start the overlap filtering process by running the following command:

```bash
ros2 run multi_lidar_registration overlap_filtering
```

After running the command, follow these steps:

1. **Start `rviz2` and visualize the following topics**:
   - **`filtered_target`**: Contains the point cloud data after filtering.
   - **`transformed_source`**: Contains the point cloud data after it has been transformed according to the filtering parameters.
   - **`start_filtering_marker`**: Visualizes the start line of the filtering region (Red line).
   - **`max_filtering_marker`**: Visualizes the right angle offset of the filtering region (Blue line).
   - **`min_filtering_marker`**: Visualizes the left angle offset of the filtering region (Green line).

2. Once the topics are visualized in `rviz2`, return to the terminal and use the following keyboard controls to adjust the filtering settings in real-time:

   - **Real-time Adjustments**:
     - `Up`, `Down`, `Left`, `Right`: Move the start point of the FOV.
     - `q`, `e`: Adjust the start angle (Red line).
     - `a`, `d`: Adjust the right angle (Blue line).
     - `z`, `c`: Adjust the left angle (Green line).
     - `n`: Add a new filtering range.
     - `b`, `m`: Switch between multiple filtering ranges.
     - `s`: Save the filtering range settings to `filter_range.txt` in the `install/share/multi_lidar_registration/keypoints/` directory.

3. **Filtering Process**:
   - The overlap filtering mechanism allows users to define specific regions within the LiDAR's field of view (FOV) to be filtered out. This tool gives users full control over which areas to filter, removing problematic data from the target LiDAR's point cloud.

This ensures that you can accurately control and visualize the overlap filtering process in real-time, leading to more precise point cloud data for further analysis or registration.