# Motion Control of Self-Driving Car for Trajectory Tracking

![Github Stars](https://badgen.net/github/stars/Tinker-Twins/Self_Driving_Car_Trajectory_Tracking?icon=github&label=stars)
![Github Forks](https://badgen.net/github/forks/Tinker-Twins/Self_Driving_Car_Trajectory_Tracking?icon=github&label=forks)

## Project Overview

<p align="justify">
This project was focused on control of an autonomous vehicle for trajectory tracking using CARLA Simulator. Various control algorithms were implemented in Python for accomplishing the task of lateral and longitudinal control of the vehicle.
</p>

Following is the list of implemented controllers:
- Lateral Controllers:
  - Bang-Bang Controller
  - PID Controller
  - Pure-Pursuit Controller
  - Stanley Controller
  - Proximally Optimal Predictive (POP) Controller
- Longitudinal Controllers:
  - PID Controller
  - Adaptive Longitudinal Controller (ALC)

|                  | **ALC**                                                                                                              | **PID**                                                                                                              |
|------------------|----------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------|
| **Bang Bang**    | ![](https://github.com/Tinker-Twins/Self_Driving_Car_Trajectory_Tracking/blob/main/Media/ALC%20-%20Bang-Bang.gif)    | ![](https://github.com/Tinker-Twins/Self_Driving_Car_Trajectory_Tracking/blob/main/Media/PID%20-%20Bang-Bang.gif)    |
| **PID**          | ![](https://github.com/Tinker-Twins/Self_Driving_Car_Trajectory_Tracking/blob/main/Media/ALC%20-%20PID.gif)          | ![](https://github.com/Tinker-Twins/Self_Driving_Car_Trajectory_Tracking/blob/main/Media/PID%20-%20PID.gif)          |
| **Pure Pursuit** | ![](https://github.com/Tinker-Twins/Self_Driving_Car_Trajectory_Tracking/blob/main/Media/ALC%20-%20Pure-Pursuit.gif) | ![](https://github.com/Tinker-Twins/Self_Driving_Car_Trajectory_Tracking/blob/main/Media/PID%20-%20Pure-Pursuit.gif) |
| **Stanley**      | ![](https://github.com/Tinker-Twins/Self_Driving_Car_Trajectory_Tracking/blob/main/Media/ALC%20-%20Stanley.gif)      | ![](https://github.com/Tinker-Twins/Self_Driving_Car_Trajectory_Tracking/blob/main/Media/PID%20-%20Stanley.gif)      |
| **POP**          | ![](https://github.com/Tinker-Twins/Self_Driving_Car_Trajectory_Tracking/blob/main/Media/ALC%20-%20POP.gif)          | ![](https://github.com/Tinker-Twins/Self_Driving_Car_Trajectory_Tracking/blob/main/Media/PID%20-%20POP.gif)          |

## File Description

- `Waypoints.txt` hosts the reference trajectory of the entire mission.
- `Controller.py` is used to implement the lateral and longitudinal controllers for tracking the trajectory.
- `Drive.py` hosts the simulation parameters, connects with the simulator and runs the entire motion control pipeline for autonomous trajectory tracking.
- `Live_Plotter.py` generates and updates plots of the vehicle states and trajectory in real-time.
- `Controller Performance Analysis.ipynb` is used to analyze the controller performance in terms of tracking metrics and latency.
- `Results` directory hosts the results of a complete trajectory tracking mission in form of plots and log files.

## Usage

**1. Install [CARLA Simulator](https://carla.org/) [Tested with CARLA v0.8.4]**

**2. Navigate to CARLA directory, for exampe:**

*Windows:*

```bash
> cd C:/CARLA
```

*Ubuntu:*

```
$ cd $HOME/CARLA
```

**3. Launch the CARLA Simulator application, for example:**

*Windows:*

```bash
> CarlaUE4.exe /Game/Maps/RaceTrack -carla-server -benchmark -fps=20 -carla-no-hud
```

*Ubuntu:*

```bash
$ ./CarlaUE4.sh /Game/Maps/RaceTrack -carla-server -benchmark -fps=20 -carla-no-hud
```

**4. Run the Python client script while choosing the appropriate longitudinal and lateral control algorithms:**

*Windows:*

```bash
> python Drive.py --Longitudinal-Controller: {PID, ALC} --Lateral-Controller: {BangBang, PID, PurePursuit, Stanley, POP}
```

*Ubuntu:*

```bash
$ python Drive.py --Longitudinal-Controller: {PID, ALC} --Lateral-Controller: {BangBang, PID, PurePursuit, Stanley, POP}
```

## Citation

- We encourage you to cite the [following chapter](https://arxiv.org/abs/2011.08729) when using this repository for your research:

  ```bibtex
  @eprint{Control-Strategies-2021,
        title={Control Strategies for Autonomous Vehicles}, 
        author={Chinmay Vilas Samak and Tanmay Vilas Samak and Sivanathan Kandhasamy},
        year={2021},
        eprint={2011.08729},
        archivePrefix={arXiv},
        primaryClass={cs.RO}
  }
  ```

  This work has been published as 2nd Chapter of the book entitled **Autonomous Driving and Advanced Driver-Assistance Systems (ADAS): Applications, Development, Legal Issues, and Testing.** The publication can be found on [CRC Press](https://www.taylorfrancis.com/books/edit/10.1201/9781003048381/autonomous-driving-advanced-driver-assistance-systems-adas-lentin-joseph-amit-kumar-mondal).


- We specifically encourage you to cite the [following paper](https://arxiv.org/abs/2103.13240) when using the POP control algorithm for your research:

  ```bibtex
  @eprint{POP-Controller-2021,
        title={Proximally Optimal Predictive Control Algorithm for Path Tracking of Self-Driving Cars}, 
        author={Chinmay Vilas Samak and Tanmay Vilas Samak and Sivanathan Kandhasamy},
        year={2021},
        eprint={2103.13240},
        archivePrefix={arXiv},
        primaryClass={cs.RO}
  }
  ```

  This work has been published in **AIR2021: Advances in Robotics - 5th International Conference of The Robotics Society.** The publication can be found on [ACM Digital Library](https://dl.acm.org/doi/10.1145/3478586.3478632).
