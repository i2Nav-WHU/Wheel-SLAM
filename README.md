# Wheel-SLAM: Simultaneous Localization and Terrain Mapping Using One Wheel-mounted IMU

[Wheel-SLAM](https://arxiv.org/pdf/2211.03174.pdf) is a SLAM solution using only one low-cost wheel-mounted IMU (Wheel-IMU). Extended from our previous studies on [Wheel-INS](https://github.com/i2Nav-WHU/Wheel-INS), a Wheel-IMU based dead reckoning system, we propose to exploit the environmental perception ability of the Wheel-IMU to enable loop closure detection in Wheel-INS. To be specific, we use the road bank angles (mirrored by the robot roll angles estimated by Wheel-INS) as terrain features for loop closure detection. The system is implemented with a Rao-Blackwellized particle filter where each particle maintains its own robot state and terrain map. The environment is represented as a 2D grid map where each grid encodes the road bank angle indicated by the robot roll angle at that position. The weights of particles are updated according to the difference between the current roll estimates and the value retrieved from the grid map.

## Introduction
### Code
The input of Wheel-SLAM is the odometry results from Wheel-INS. The entrance of the code is "***wheelslam_main.m***". The parameters can be found in "***config202107311.m***".

### Dataset
We also provide a set of data with ground truth for reproducing the results in our paper. The robot trajectory estimated by Wheel-INS have been transformed into a dead reckoning format, e.g., the travel distance and the heading change of the robot. Each line of the dataset file has four columns, time stamp, traveled distance, heading increment, and roll angle. Plese refer to the ***data*** folder.

## Citations
If you find our study helpful to your research, please consider to cite our Wheel-SLAM paper
```bibtex
@ARTICLE{wu2022ral,
  title={{Wheel-SLAM}: Simultaneous Localization and Terrain Mapping Using One Wheel-mounted IMU},
  author={Wu, Yibin and Kuang, Jian and Niu, Xiaoji and Behley, Jens and Klingbeil, Lasse and Kuhlmann, Heiner},
  journal={IEEE Robotics and Automation Letters},
  doi={10.1109/LRA.2022.3226071},
  year={2022}
}
```
or/and our Wheel-INS paper.
```bibtex
@ARTICLE{niu2021tvt,
  author={Niu, Xiaoji and Wu, Yibin and Kuang, Jian},
  journal={IEEE Transactions on Vehicular Technology}, 
  title={{Wheel-INS}: A Wheel-Mounted MEMS IMU-Based Dead Reckoning System}, 
  year={2021},
  volume={70},
  number={10},
  pages={9814-9825},
  doi={10.1109/TVT.2021.3108008}
}
```

## Additional Experimental Results
Here we provide some supplementary results and explanation to our paper. The experiments were carried out in the campus of Wuhan University, including five sequences. The street view of the experimental trajectories are shown as following.

*The five experimental trajectories.*
![allSequences](https://user-images.githubusercontent.com/25290921/200004526-77b7a9a9-8956-4d24-84b9-a802d93b79f9.png)

*The street view of the experimental trajectories.*
![streetview](https://user-images.githubusercontent.com/25290921/199988955-abba5ac5-fc29-4987-beeb-5464faeb1374.png)

Following figures show the positioning results of Wheel-SLAM overlaying on Google earth. Some areas have been zoomed in. It can be observed that Wheel-SLAM matches the ground truth well in general, despite few error drift. We can also see that in some areas, e.g., the enlarged picture at the top of Fig. 1, the drifted trajectory is pulled back thanks to the successful loop closure detection in Wheel-SLAM. We argue that we only correct current robot state but not history trajectory after loop closure in Wheel-SLAM. The positioning errors accumulated when loop closure is not detected still affects the overall accuracy. However, this does not affect the conclusion that the roll angle of the vehicle can be used to detect the closed loop so as to control the position drift of Wheel-INS effectively.

*Seq. 1.*
![seq1](https://user-images.githubusercontent.com/25290921/199610817-cd814c78-fb24-46f8-898a-fbab6b73a29d.png)

*Seq. 2.*
![seq2](https://user-images.githubusercontent.com/25290921/199610836-1514a8b2-db02-47a8-80af-d8e9c94b2438.png)

*Seq. 3.*
![seq3](https://user-images.githubusercontent.com/25290921/199610853-581bbee4-85bc-4583-bf72-6cecb5b2f20a.png)

*Seq. 4.*
![seq4](https://user-images.githubusercontent.com/25290921/199610750-8ed17084-9eef-4436-958f-efdee9b30034.png)

*Seq. 5.*
![seq5](https://user-images.githubusercontent.com/25290921/199610768-1c604bbd-ebec-44a8-8e5e-c39917b96d49.png)

## Acknowledgement
We would like to thank [Tim Bailey](https://github.com/OpenSLAM-org/openslam_bailey-slam) for sharing his implementation of [FastSLAM 1.0](http://robots.stanford.edu/papers/montemerlo.fastslam-tr.pdf) and [2.0](http://robots.stanford.edu/papers/Montemerlo03a.pdf). 