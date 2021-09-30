# **MOBATSim**

MOBATSim (Model-based Autonomous Traffic Simulation Framework) is a framework based on MATLAB® and Simulink® that allows users to develop automated driving algorithms and assess their safety and performance. By running a traffic simulation, the safety of the implemented component or algorithm can be measured on both the vehicle level and the traffic level, supported by 2D and 3D visualization options.

[![MOBATSim - MOBATSim](https://img.shields.io/static/v1?label=MOBATSim&message=MOBATSim&color=96D1AA&logo=github)](https://github.com/MOBATSim/MOBATSim)
[![Contributors - MOBATSim](https://img.shields.io/github/contributors/MOBATSim/MOBATSim)](https://github.com/MOBATSim/MOBATSim/graphs/contributors)
[![Pull Requests - MOBATSim](https://img.shields.io/github/issues-pr-closed/MOBATSim/MOBATSim?color=g&logoColor=0)](https://github.com/MOBATSim/MOBATSim/pulls)
[![Commit Activity - MOBATSim](https://img.shields.io/github/commit-activity/m/MOBATSim/MOBATSim)](https://github.com/MOBATSim/MOBATSim/pulse)
[![GitHub tag](https://img.shields.io/github/tag/MOBATSim/MOBATSim?include_prereleases=&sort=semver&color=96D1AA)](https://github.com/MOBATSim/MOBATSim/releases/)
[![Commit Activity - MOBATSim](https://img.shields.io/github/commits-since/MOBATSim/MOBATSim/v2.0.0?color=%23ccff00&label=commits%20since%20last%20release)](https://github.com/MOBATSim/MOBATSim/commits)



**Consider [starring our GitHub Repositories](https://github.com/MOBATSim/MOBATSim/stargazers) and [subscribing to our MOBATSim YouTube Channel](https://www.youtube.com/c/MOBATSim) to support us!**


[![stars - MOBATSim](https://img.shields.io/github/stars/MOBATSim/MOBATSim?style=social)](https://github.com/MOBATSim/MOBATSim/stargazers)
[![forks - MOBATSim](https://img.shields.io/github/forks/MOBATSim/MOBATSim?style=social)](https://github.com/MOBATSim/MOBATSim/fork)
[![YouTube](https://img.shields.io/youtube/channel/views/UCVP9SDdAH_TcXCfGsGFQ09Q?style=social)](https://www.youtube.com/c/MOBATSim)
[![YouTube](https://img.shields.io/youtube/channel/subscribers/UCVP9SDdAH_TcXCfGsGFQ09Q?style=social)](https://www.youtube.com/c/MOBATSim)

[![View MOBATSim_OpenSource on File Exchange](https://www.mathworks.com/matlabcentral/images/matlab-file-exchange.svg)](https://www.mathworks.com/matlabcentral/fileexchange/78444-mobatsim)
[![Download](https://img.shields.io/badge/MATLAB-R2020b-orange)](https://github.com/MOBATSim/MOBATSim/releases/tag/v1.0.2)
[![Cite our paper](https://img.shields.io/badge/Cite%20our%20paper-BibTex-blueviolet)](#citation)

**Autonomous Vehicle Modeling and Simulation in Simulink Tutorials**

[![MustafaSaraoglu - AutonomousVehicleModeling](https://img.shields.io/static/v1?label=MustafaSaraoglu&message=AutonomousVehicleModeling&color=orange&logo=github)](https://github.com/MustafaSaraoglu/AutonomousVehicleModeling)
[![stars - AutonomousVehicleModeling](https://img.shields.io/github/stars/MustafaSaraoglu/AutonomousVehicleModeling?style=social)](https://github.com/MustafaSaraoglu/AutonomousVehicleModeling/stargazers)
[![forks - AutonomousVehicleModeling](https://img.shields.io/github/forks/MustafaSaraoglu/AutonomousVehicleModeling?style=social)](https://github.com/MustafaSaraoglu/AutonomousVehicleModeling/fork)
[![Follow](https://img.shields.io/github/followers/MustafaSaraoglu?style=social)](https://github.com/MustafaSaraoglu)
[![YouTube](https://img.shields.io/youtube/likes/MS66nlDOmRY?style=social&withDislikes)](https://www.youtube.com/watch?v=MS66nlDOmRY&list=PLNNL3443z4lHTmBFrrur6aYhJnwvITqWM)


<img src="img/logo_big.jpg" alt="Combined Image" />

# Table of contents
1. [Introduction](#introduction)
2. [Key Features of MOBATSim](#keyfeatures)
3. [Requirements](#requirements)
4. [Citation](#citation)
5. [Contributing to MOBATSim](#contribution)
6. [Release Notes](#releasenotes)
7. [Getting Started](#gettingstarted)

<a name="introduction"></a>
## Introduction

Automated driving systems tend to be more critical and sophisticated in the nearest future. The functional safety assessment for these systems becomes an urgent necessity for the transition to full autonomy. Testing these functions consisting of decision and control algorithms with many variables and parameters in a unified manner is a daunting task. Threat assessment has to be made for vehicles to avoid hazardous situations actively. This requires analyzing complex operational profiles such as routing, intersection management, and collision prediction in an environment where multiple vehicles are in different positions and traveling at different speeds. There is a need for a comprehensive traffic simulation framework that models the functionality of the vehicles and the interactions between them.

More detailed information about the scientific papers related to MOBATSim can be found on our [website](https://mobatsim.com/),

or you can visit our YouTube Channel where we publish the latest updates with tutorial videos:
[![YouTube Channel](http://img.youtube.com/vi/3Wz3D1v-lL8/0.jpg)](https://www.youtube.com/c/MOBATSim)

<a name="keyfeatures"></a>

## **Key Features of MOBATSim** 

* All the scripts, class files, and functions used in MOBATSim are open for editing. Users can control all the vehicles, traffic management algorithms, and the map.
* Each vehicle is considered as an agent, and the traffic is simulated as a closed-loop multi-agent system. The vehicles generate their trajectories during the simulation according to the states and intentions of the other vehicles around in the environment. This feature also allows reactive planning algorithms to be developed and tested.
* Users can either develop an algorithm or a controller for a single vehicle (usually referred to as the ego vehicle) or different implementations for different vehicles simultaneously.
* Full control over all the states regarding the simulation allows for fault injection and error propagation analysis. States can be easily manipulated during the simulation by implementing either Simulink fault injection blocks or code snippets in MATLAB System Block functions.
* MOBATSim can be used for benchmarking control and decision algorithms regarding safety and performance on different abstraction levels such as component level, vehicle level, and traffic level.
* Object-oriented programming structure (MATLAB Classes) combined with a block diagram environment (Simulink) allows a flexible framework suitable for collaboration.
* Data logging can be extended to states and signals of interest other than the default vehicle states used by the post-simulation 3D visualization.
* The compatible data structure allows for various post-simulation visualization options (e.g., Unreal Engine 4 support, Bird's Eye View Scope, or Simulink 3D Animation).
* MOBATSim's compatible map structure allows road network extensions through a user-friendly interface using the Driving Scenario Designer app.

<a name="requirements"></a>

## **MATLAB Version and Toolbox Requirements** 

MOBATSim is continuously updated with the latest version of MATLAB®. Therefore the requirement is **MATLAB R2021a** or **MATLAB R2020b**. The following toolboxes are required for running MOBATSim:

* Simulink® and Stateflow®
* Automated Driving Toolbox™
* Robotics System Toolbox™
* Control System Toolbox™
* Deep Learning Toolbox™
* Symbolic Math Toolbox™
* Model Predictive Control Toolbox™  (only if MPC-Cruise Controller Block is used)
* Simulink 3D Animation Toolbox™ (only required for the 3D Animation Virtual World)

<a name="citation"></a>
## Authors and Contact 

**Main Author:** Mustafa Saraoğlu

**Contributors:** Johannes Pintscher, Laura Slabon, Qianwei Yang, Qihang Shi, Wenkai Wu, Maoxuan Zhao, Erik Noack, Fabian Hart, Müjdat Korkmaz, Marta Valdes Martin

Message us via the [contact form](https://mobatsim.com/contact/) on our website!

Copyright © 2017 MOBATSim.

### Please Cite Our Related Paper as:

Saraoglu, M., Morozov, A., & Janschek, K. (2019). MOBATSim: MOdel-Based Autonomous Traffic Simulation Framework for Fault-Error-Failure Chain Analysis. IFAC-PapersOnLine, 52(8), 239–244. Elsevier BV. Retrieved from https://doi.org/10.1016%2Fj.ifacol.2019.08.077

BibTex:
```
@article{MOBATSim,
                title = {{MOBATSim}: {MOdel}-Based Autonomous Traffic Simulation Framework for Fault-Error-Failure Chain Analysis},
                journal = "IFAC-PapersOnLine",
                volume = "52",
                number = "8",
                pages = "239 - 244",
                year = "2019",
                note = "10th IFAC Symposium on Intelligent Autonomous Vehicles IAV 2019",
                issn = "2405-8963",
                doi = "https://doi.org/10.1016/j.ifacol.2019.08.077",
                url = "http://www.sciencedirect.com/science/article/pii/S2405896319304100",
                author = "Mustafa Saraoglu and Andrey Morozov and Klaus Janschek",
                keywords = "Autonomous driving, Fault injection, Error propagation, Safety analysis, Traffic simulator",
                }
```
<a name="contribution"></a>
## Contributing to MOBATSim 

If you find MOBATSim useful and you would like to improve it by implementing your own automated driving algorithms:
1. **"Fork"** the repository from the top right corner.
2. Go to your **forked repository** and switch to the **development branch**.
3. Make your changes in the development branch. Make sure your contributions fit the format in terms of coding or input/output properties of the Simulink blocks.
4. To make sure that your changes work, you should run `MOBATSimAutoTesting.m` in `src/Scripts` folder and if you get `passed` from all the tests, you should **"commit"** and then **"Push"** to your forked repository.
5. Then if you would like to contribute, send a **"Pull Request"** to the corresponding branch on the MOBATSim/MOBATSim repository.
6. Once it is reviewed, it will be **approved** or **changes will be requested along with the comments of the reviewer** regarding the issue with your **Pull Request**.

We would like to encourage everyone who would like to contribute, so you can also contact us for a more detailed explanation of the structure!

<a name="releasenotes"></a>
## Release Notes - Version 2.0

* New ways to visualize your driving scenario: **Unreal Engine 4** support via **DrivingScenarioDesigner App**, **Bird's Eye View**.
* New vehicle **kinematic bicycle models** taken from the **Automated Driving Toolbox library**.
* A more detailed road structure with actual units as meters and double lane roads.
* New lateral controllers: **Stanley lateral controller** for common vehicles, **Pure Pursuit lateral controller** for the Ego Vehicle (Vehicle 2).
* Implementation of **Frenet Coordinate system** for local trajectory planning.
* Implementation of **lane-changing maneuver on double lane roads** (at the moment only allowed for Pure Pursuit controller - Ego Vehicle)
* An improved coding structure using superclasses, name-value pairs to also enhance the flexibility of MOBATSim and also code optimization using vectorizations and memory preallocations to increase the performance.
* Detailed documentation for the people who are interested and would like to understand and contribute to MOBATSim
* Bonus content: **3D Animation World** with the new Dinosaur Park.

## Known Issues and Bugs

* Vehicles are not allowed to start or finish on the intersection points to avoid congestion.
* **Bird's Eye View** or **DrivingScenarioDesigner APP** might work slowly because of the size of the road network.
* Some road merges do not have safety guarantees which means that a vehicle just merging another road at the same time with another vehicle or there is a stopping vehicle at the merging point of the joining road may cause collisions.
* Changing the default sample time value of `0.02` or playing with different **Simulink Solver** options other than `auto` may cause unexpected behavior.

<a name="gettingstarted"></a>
## Getting Started

MOBATSim has a project file that includes the Simulink files and their paths. The project can be opened by double-clicking on `MOBATSim.prj` and a GUI will appear, which can be used to start the simulation. Simply click on `Start Simulation` and wait for the simulation to start.



First it would be best if you [fork the MOBATSim repository](https://github.com/MOBATSim/MOBATSim/fork) and then **clone** it to your computer. After opening the **MOBATSim** folder please refer to the live script file `GettingStarted.mlx` for more detailed documentation.
