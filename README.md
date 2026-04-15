## CalibAnything for Riibotics

This package provides an automatic and target-less LiDAR-camera extrinsic calibration method using Segment Anything Model. The related paper is [Calib-Anything: Zero-training LiDAR-Camera Extrinsic Calibration Method Using Segment Anything](https://arxiv.org/abs/2306.02656). For more calibration codes, please refer to the link <a href="https://github.com/PJLab-ADG/SensorsCalibration" title="SensorsCalibration">SensorsCalibration</a>.

We adds some modifications on code and simple program scripts for data processing to perform calibration between the camera on the forklift of Riibotics and the lidar, and provides guidelines for good calibration.

### Basic operation guideline
1. This code is not available on PCs without GPUs and is recommended for operation on personal laptops or desktops with plenty of process performance.

2. Please select an appropriate scene for performing calibration. At this time, LiDAR and camera should have similar views as much as possible. Recognize that calibration is impossible if the overlapping area is too small. The criteria for a good view are as follows: The more planes with various depths, the better. Auto segment algorithms automatically deploy objects to make the segment work, and avoid items that do not. Also, it's more advantageous to choose an indoor environment than an outdoor one. Avoid areas with infinite depth, such as windows.

3. Get at least 30 point clouds and camera images while still in that scene. The current program supports either a combination of .pcd/.png or a rosbag form.

4. Use the dataset processing script to process the data appropriately for the program and place it in the appropriate position.

5. You must set the default settings (calib.json) for the program. (Intrinsic / Initial Extrinsic, search_range, min_plane_point_num, cluster_tolerance ...)

6. Validate performance based on the resulting image.

### Prerequisites for Ubuntu 22.04 / ROS2 Humble / RTX 4060
- numpy (==2.4.4)
- opencv-python (==4.13.0.92)
- torch (==2.9.1)
- torchvision (==0.24.1)
- pycocotools (=2.0.11)
- onnx (==1.16.1)
- onnxruntime (==1.24.4)
- matplotlib
- cv-bridge (==4.1.0)
- rosbag2_py (==0.26.9)
- rclpy (==7.1.6)
- rosidl_runtime_py (==0.13.1)
- sensor_msgs_py (==5.3.6)

- PCL 1.10
- Eigen3
- OpenCV
- jsoncpp

### Compile
```shell
cd CalibAnything
mkdir build
cd build
cmake ..
make
```

### Run example for Riibotics

# Default README.md
## CalibAnything

This package provides an automatic and target-less LiDAR-camera extrinsic calibration method using Segment Anything Model. The related paper is [Calib-Anything: Zero-training LiDAR-Camera Extrinsic Calibration Method Using Segment Anything](https://arxiv.org/abs/2306.02656). For more calibration codes, please refer to the link <a href="https://github.com/PJLab-ADG/SensorsCalibration" title="SensorsCalibration">SensorsCalibration</a>.

## Prerequisites
- pcl 1.10
- opencv
- eigen 3

## Compile
```shell
git clone https://github.com/OpenCalib/CalibAnything.git
cd CalibAnything
# mkdir build
mkdir -p build && cd build
# build
cmake .. && make
```

## Run Example
We provide examples of two dataset. You can download the processed data at [Google Drive](https://drive.google.com/file/d/1OCtbIGilLOBnHzY5VNHqRZzbxXj3xiXc/view?usp=drive_link) or [BaiduNetDisk](https://pan.baidu.com/s/1qAt7nYw5hYoJ1qrH0JosaQ?pwd=417d):
```
# baidunetdisk
Link: https://pan.baidu.com/s/1qAt7nYw5hYoJ1qrH0JosaQ?pwd=417d 
Code: 417d
```

Run the command:
```shell
cd CalibAnything
./bin/run_lidar2camera ./data/kitti/calib.json # kitti dataset
./bin/run_lidar2camera ./data/nuscenes/calib.json # nuscenes dataset
```

## Test your own data

### Data collection

- Several pairs of time synchronized RGB images and LiDAR point cloud (intensity is needed). One pair of data can also be used to calibrate, but the results may be ubstable.
- The intrinsic of the camera and the initial guess of the extrinsic.

### Preprocessing

#### Generate masks

Follow the instructions in [Segment Anything](https://github.com/facebookresearch/segment-anything) and generate masks of your image.

1. First download a model checkpoint. You can choose [vit-l](https://dl.fbaipublicfiles.com/segment_anything/sam_vit_l_0b3195.pth).

2. Install SAM
```shell
# environment: python>=3.8, pytorch>=1.7, torchvision>=0.8

git clone git@github.com:facebookresearch/segment-anything.git
cd segment-anything; pip install -e .
pip install opencv-python pycocotools matplotlib onnxruntime onnx
```

3. Run
```shell
python scripts/amg.py --checkpoint <path/to/checkpoint> --model-type <model_type> --input <image_or_folder> --output <path/to/output>

# example(recommend parameter)
python scripts/amg.py --checkpoint sam_vit_l_0b3195.pth --model-type vit_l --input ./data/kitti/000000/images/  --output ./data/kitti/000000/masks/ --stability-score-thresh 0.9 --box-nms-thresh 0.5 --stability-score-offset 0.9
```

#### Data folder
The hierarchy of your folders should be formed as:
```
YOUR_DATA_FOLDER
├─calib.json
├─pc
|   ├─000000.pcd
|   ├─000001.pcd
|   ├─...
├─images
|   ├─000000.png
|   ├─000001.png
|   ├─...
├─masks
|   ├─000000
|   |   ├─000.png
|   |   ├─001.png
|   |   ├─...
|   ├─000001
|   ├─...

```

#### Processed masks

For large masks, we only use part of it near the edge.
```shell
python processed_mask.py -i <YOUR_DATA_FOLDER>/masks/ -o <YOUR_DATA_FOLDER>/processed_masks/
```

#### Edit the json file

<details><summary>Content description</summary>

- `cam_K`: camera intrinsic matrix
- `cam_dist`: camera distortion coefficient. `[k1, k2, p1, p2, p3, ...]`, use the same order as [opencv](https://amroamroamro.github.io/mexopencv/matlab/cv.initUndistortRectifyMap.html)
- `T_lidar_to_cam`: initial guess of the extrinsic
- `T_lidar_to_cam_gt`: ground-truth of the extrinsic (Used to calculate error. If not provided, set "available" to false)
- `img_folder`: the path to images
- `mask_folder`: the path to masks
- `pc_folder`: the path to point cloud
- `img_format`: the suffix of the image
- `pc_format`: the suffix of the point cloud (support pcd or kitti bin)
- `file_name`: the name of the input images and point cloud
- `min_plane_point_num`: the minimum number of point in plane extraction
- `cluster_tolerance`: the spatial cluster tolerance in euclidean cluster (set larger if the point cloud is sparse, such as the 32-beam LiDAR)
- `search_num`: the number of search times
- `search_range`: the search range for rotation and translation
- `point_range`: the approximate height range of the point cloud projected onto the image (the top of the image is 0.0 and the bottom of the image is 1.0)
- `down_sample`: the point cloud downsample voxel size (if don't need downsample, set the "is_valid" to false)
- `thread`: the number of thread to reduce calibration time
</details>

### Calibration
```shell
./bin/run_lidar2camera <path-to-json-file>
```

## Output
- initial projection: `init_proj.png`, `init_proj_seg.png`
- gt projection: `gt_proj.png`, `gt_proj_seg.png`
- refined projection: `refined_proj.png`, `refined_proj_seg.png`
- refined extrinsic: `extrinsic.txt`

## Citation
If you find this project useful in your research, please consider cite:
```
@misc{luo2023calibanything,
      title={Calib-Anything: Zero-training LiDAR-Camera Extrinsic Calibration Method Using Segment Anything}, 
      author={Zhaotong Luo and Guohang Yan and Yikang Li},
      year={2023},
      eprint={2306.02656},
      archivePrefix={arXiv},
      primaryClass={cs.CV}
}
```
