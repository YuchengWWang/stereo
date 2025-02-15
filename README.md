# [Stereo calibration and mapping]

Code for entire stereo calibration and mapping process: 
- Calibration (intrinsic, extrinsic)
- Distortion compensation
- Epipolar calibration
- Stereo matching & disparity
- Depth
- Point cloud

# Demo
- <p><b>left and right image<b><p>
<img src="./left_demo.jpg" alt="left" style="width: 45%;">          <img src="./right_demo.jpg" alt="right" style="width: 45%;">
- <p><b>disparity and depth map<b><p>
<img src="./disparity.jpg" alt="disparity" style="width: 45%;">          <img src="./depth.jpg" alt="depth" style="width: 45%;">

## 🛠️ Preparation

```
mkdir -p images/oringin_left
mkdir -p images/oringin_right
mkdir -p image_undistort/left_image_undistort
mkdir -p image_undistort/right_image_undistort
mkdir calib_images_left
mkdir calib_images_right
mkdir build
cd build
cmake ..
make
```


## 📷 Take photos

Connect you computer with a pair of cameras fixes on the rig. Run the following command and press space to take chessboard photos from different perspectives. 
```
cd build
./photo
```

You can also directly put your images into ./images folder as:
```
└── images/
    ├── origin_left/
    │   ├── image_0.jpg
    │   ├── image_1.jpg
    │   ├── image_2.jpg
    │   └── ...
    └── origin_right/
        ├── image_0.jpg
        ├── image_1.jpg
        ├── image_2.jpg
        └── ...
```


## 🤖 Intrinsic calibration

Run the following line to get you intrinsic and distortion params.

```
./intrinsic
```


## 👾 Distortion compensation

Run the following line to compensate the distort of images.

```
./undistort
```


## ⚙️ Extrinsic and Epipolar calibration

Run the following line for extrinsic and epipolar calibration.

```
./epipolar_calibration
```


## 🚀 Stereo matching & Disparity & Depth & Point cloud

Run the following line to get your disparity, depth and point cloud.

```
./stereo
```

## 🧐 Point cloud visualization

Run the following line to visualize your point cloud (you should have open3d installed first).

```
pip install open3d
cd ..
python ./cloud_viz.py
```

![img](./pointcloud.gif)

