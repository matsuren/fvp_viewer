# FVP Viewer
Viewer for [Free Viewpoint Image Generation System using Fisheye Cameras and a Laser Rangefinder for Indoor Robot Teleoperation](http://dx.doi.org/10.1186/s40648-020-00163-4). 

Please check [our project page](https://matsuren.github.io/fvp) for details.


If you use this code for your academic research, please cite the following paper.
```
@article{komatsu2020fvp,
  title={Free viewpoint image generation system using fisheye cameras and a laser rangefinder for indoor robot teleoperation},
  author={Komatsu, Ren and Fujii, Hiromitsu and Tamura, Yusuke and Yamashita, Atsushi and Asama, Hajime},
  journal={ROBOMECH Journal},
  volume={7},
  number={15},
  pages={1--10},
  year={2020},
  publisher={Springer}
}
```


## Environment
Visual Studio 2015, 2017, 2019

## Installation
Please install OpenCV first.
After that, run `build_script_win.bat` for Visual Studio 2019. 
(If you use Visual Studio 2015 or 2017, please change `GENERATOR_NAME` in `build_script_win.bat`.) 

## Calibration
- Attach four fisheye cameras and a LRF on a robot.
- Estimate the poses of the cameras, the LRF, and the robot model using [our calibration program](https://github.com/matsuren/fvp_calibration).

## Generate Free viewpoint images
The structure of `data` folder is the followings:

    data/
    ├── calib_results_{i}.txt (The intrinsic parameter of the fisheye cameras estimated by OcamCalib
    ├── img{i}.jpg (Fisheye images that were used for our calibration program)
    ├── urg_xy.csv (LRF measurement that was used for our calibration program)
    ├── final_camera_poses.yml (Fisheye camera poses estimated using our calibration program)
    ├── lrf_align_matrix.yml (LRF pose estimated using our calibration program)
    └── robot_align_matrix.yml (Robot model pose estimated using our calibration program)

You can modify the serial number of the cameras, the COM port for LRF,  etc. in `config_FVP_parameters.json`.

After build, run `viewer` to generate free viewpoint images.