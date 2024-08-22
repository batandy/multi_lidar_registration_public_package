# Multi LiDAR Registration
<p align = "center">
<img src= "https://github.com/batandy/multi_lidar_registration_public_package/blob/main/docs/aligned_four_lidars.png" alt="aligned four lidars" height="470">
</p> 

# readme 작성중~

## Descriptions
This project is a ROS2-Humble-based application for processing and registering LiDAR point cloud data from multiple sensors. It includes functionalities for filtering, transforming point clouds from various LiDAR models like "LIVOX MID360", "LIVOX AVIA" and "Velodyne 16". The processed point clouds are used to extract keypoints and align them for further analysis or usage in robotics applications.

## Table of Contents
- [Installation](#installation)
- [Usage](#usage)
- [Pre-requisites](#pre-requisites)
- [Configuration](#configuration)
- [Registration](#Registration)
- [Overlap Filtering](#Overlap-Filtering)

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

<p align = "center">
<img src= "https://github.com/batandy/multi_lidar_registration_public_package/blob/main/docs/calibration_target.jpg" alt="aligned four lidars" height="470">
</p> 

For effective point cloud registration, a physical calibration target is required. The target should have the following specifications:

- **Top Section**: A circular disk with a diameter of 30 cm.
- **Bottom Section**: A frustum-shaped lava cone with a height of 84 cm, a top radius of 17 cm, and a bottom radius of 7 cm.
- **Reflective Film**: The reflective film should be applied only to the circular top section and the frustum-shaped bottom section.


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
  #velodyne 16-channel -> vlp_callback

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
- **opposite**: Set to true if the registered LiDAR needs to be mirrored horizontally. This is primarily necessary when two LiDARs are facing opposite directions, as the left side from the perspective of LiDAR 1 corresponds to the right side from the perspective of LiDAR 2, and vice versa. Mirroring ensures consistent alignment between the data from both LiDARs.

#### Running Keypoints Extraction

After you have configured the `registration_config.yaml` file according to your LiDAR setup, you can start the keypoint extraction process by executing the following command:

```bash
ros2 launch multi_lidar_registration registration.launch.py
```

This process involves several key steps:

1. **Point Cloud Reception**:
   - The system receives point clouds from various LiDAR sensors, including Velodyne (16 channels), MID360, and AVIA.
   - These point clouds are processed using the Point Cloud Library (PCL) to prepare for keypoint extraction.

2. **Keypoint Extraction**:
   - The primary objective of this process is to extract distinctive features from the scanned point cloud data. The code uses techniques such as **Passthrough filtering**, **Voxel Grid filtering**, and **Principal Component Analysis (PCA)** for this purpose.
   - The extracted keypoints are critical for aligning or registering point clouds from different sensors or different positions of the same sensor.

   ![Keypoint Extraction Example](https://github.com/batandy/multi_lidar_registration_public_package/blob/main/docs/keypoint_ex.png)
   *This is an example of the keypoint extraction process.*

3. **Saving Keypoints**:
   - The keypoints are then saved to specific files located in the `install/share/multi_lidar_registration/keypoints/` directory as `source_keypoint.txt` and `target_keypoint.txt`.
   - These files represent the keypoints of the source and target point clouds, respectively.

4. **Additional Functionalities**:
   - The code includes a custom point type structure (`PointXYZIR`) designed to handle the ring structure (e.g., from Velodyne LiDAR). This structure includes fields for the XYZ coordinates, intensity, and ring index.
   - There are predefined physical dimensions for calibration target, which might be used for calibration or as reference points in the environment.
   - The process also appears to be capable of handling multiple LiDARs simultaneously, with flags and counters (`src_finish`, `trg_finish`, `src_cnt`, `trg_cnt`) to manage the completion status of the keypoint extraction.

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

#### Running the Point Cloud Registration

To start the registration process, execute the following command:

```bash
ros2 run multi_lidar_registration publisher
```

This command initiates the registration process using the extracted keypoints and the configuration settings specified in your setup. Here’s a detailed breakdown of what happens:

1. **Transformation Matrix Calculation**:
   - Using the keypoints extracted earlier, the system calculates a transformation matrix (RT) via Singular Value Decomposition (SVD). This matrix aligns the source point cloud with the target point cloud, providing an initial rough alignment.

2. **Refinement with GICP**:
   - After the initial alignment using keypoints, the Generalized Iterative Closest Point (GICP) algorithm is applied for precise registration. GICP refines the initial alignment by iteratively minimizing the distance between corresponding points in the source and target point clouds, leading to more accurate and precise registration.

3. **Publishing Registered Point Clouds**:
   - The registered point clouds are published to ROS topics. The specific topics depend on the value of the **publish_merged_topic** parameter:
     - **If `publish_merged_topic` is set to `true`**: 
       - The point clouds from the source and target LiDARs are combined into a single set of topics. 
       - The transformed (aligned) point cloud is published under `merged_transformed`, and the baseline alignment (initial rough alignment) is published under `merged_baseline`.
     - **If `publish_merged_topic` is set to `false`**:
       - The point clouds from the source and target LiDARs are published separately.
       - The source LiDAR’s transformed point cloud is published to `<source_topic>_transformed`, and its baseline alignment is published to `<source_topic>_baseline`.
       - Similarly, the target LiDAR’s baseline alignment is published to `<target_topic>_baseline`.
       - Here, `<source_topic>` and `<target_topic>` are placeholders that are replaced by the actual topic names you configured for your LiDARs.

4. **Overlap Filtering**:
   - The overlap filtering feature is designed to help users manage the point clouds after registration by removing specific regions from the target LiDAR's data. This functionality is particularly useful to prevent issues such as excessive points in overlapping areas, which could lead to errors during further analysis or use of the registered point clouds. Users can specify regions for filtering through the command `ros2 run multi_lidar_registration overlap_filtering`, effectively deleting points in the designated areas to improve the usability of the final point cloud data.

5. **Binary File Recording**:
   - If the `start_bin_recording` flag is set, the system records the binary data of the processed point clouds to a file, which could be useful for later analysis or debugging.

### Overlap-Filtering

The `overlap_filtering_controller.cpp` file is responsible for performing overlap filtering on LiDAR point clouds. This process allows users to adjust and apply filtering to specific overlapping regions in real-time, ensuring that unwanted data does not interfere with further analysis or application of the point clouds. Below is a detailed explanation of the overlap filtering process and the functionalities provided by this code.

![Overlap Filtering Example](path/to/your/image.png)
*This is an example of the overlap filtering process.*

To filter the overlapping regions, you can start the overlap filtering process by running the following command:

```bash
ros2 run multi_lidar_registration overlap_filtering
```
This command launches the node that handles the overlap filtering process. After running the command, follow these steps:

1. **Start `rviz2` and visualize the following topics**:
   - `filtered_target`: Contains the point cloud data after the user-defined filtering has been applied.
   - `transformed_source`: Contains the point cloud data after it has been transformed according to the filtering parameters.
   - `start_filtering_marker`: Visualizes the start line of the filtering region (represented by the `Red` line).
   - `max_filtering_marker`: Visualizes the right angle offset of the filtering region (represented by the `Blue` line).
   - `min_filtering_marker`: Visualizes the left angle offset of the filtering region (represented by the `Green` line).

2. Once the topics are visualized in `rviz2`, return to the terminal where you ran the `ros2` command and use the following keyboard controls to adjust the filtering settings in real-time:

   - **Real-time Adjustments**:
     - `Up`, `Down`, `Left`, `Right`: Move the start point of the FOV within the point cloud data.
     - `q`, `e`: Adjust the start angle for the filtering region (represented by the `Red` line).
     - `a`, `d`: Adjust the right angle of the filtering region (represented by the `Blue` line).
     - `z`, `c`: Adjust the left angle of the filtering region (represented by the `Green` line).
     - `n`: Add a new filtering range.
     - `b`, `m`: Switch between multiple filtering ranges that you have defined.
     - `s`: Save the current filtering range settings to a file named `filter_range.txt` in `install/share/multi_lidar_registration/keypoints/` directory.

3. **Filtering Process**:
   - The overlap filtering mechanism allows users to define specific regions within the LiDAR's field of view (FOV) to be filtered out. This is particularly useful for eliminating points from overlapping regions where errors or excessive data could cause issues in downstream processing.
   - Unlike automatic filtering, this tool gives the user full control over which areas to filter, allowing for precise adjustments to remove problematic data from the target LiDAR's point cloud.

