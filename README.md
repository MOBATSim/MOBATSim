# **MOBATSim**

[![View MOBATSim_OpenSource on File Exchange](https://www.mathworks.com/matlabcentral/images/matlab-file-exchange.svg)](https://www.mathworks.com/matlabcentral/fileexchange/78444-mobatsim_opensource)
[![Download](https://img.shields.io/badge/MATLAB-R2020a-brightgreen)](https://github.com/MOBATSim/MOBATSim/releases/tag/v1.0.2)
[![Download](https://img.shields.io/badge/MATLAB-R2020b-brightgreen)](https://github.com/MOBATSim/MOBATSim/releases/tag/v1.0.2)
[![View Article](https://img.shields.io/badge/DOI-https%3A%2F%2Fdoi.org%2F10.1016%2Fj.ifacol.2019.08.077-blue)](https://www.sciencedirect.com/science/article/pii/S2405896319304100)

MOBATSim (Model-based Autonomous Traffic Simulation Framework) is a simulation framework based on MATLAB and Simulink that allows users to develop automated driving algorithms to assess their safety and performance. By running a traffic simulation, the safety of the implemented component or algorithm is measured on both the vehicle level and the traffic level, supported by 2D and 3D visualization options.

<img src="img/logo_big.jpg" alt="Combined Image" />

# Table of contents
1. [Introduction](#introduction)
2. [Requirements](#requirements)
3. [Citation](#citation)
4. [Contributing to MOBATSim](#contribution)
5. [Release Notes](#releasenotes)
6. [Getting Started](#gettingstarted)

<a name="introduction"></a>
## Introduction

MOBATSim (Model-based Autonomous Traffic Simulation Framework) is a simulation framework based on MATLAB® Simulink® that allows the user to assess vehicle and traffic level safety by a 3D traffic simulation. Automated driving systems tend to be more critical and sophisticated in the nearest future. The functional safety assessment for these systems becomes an urgent necessity for the transition to full autonomy. Testing these functions consisting of decision and control algorithms with many variables and parameters in a unified manner is a daunting task. Threat assessment has to be made for vehicles to avoid hazardous situations actively. This requires analyzing complex operational profiles such as routing, intersection management, and collision prediction in an environment where multiple vehicles are in different positions and traveling at different speeds. There is a need for a comprehensive traffic simulation framework that models the functionality of the vehicles and the interactions between them.

More detailed information about the scientific papers related to MOBATSim can be found on the [website](https://mobatsim.com/)

or you can visit our YouTube Channel where we publish the latest updates with tutorial videos:
[![YouTube Channel](http://img.youtube.com/vi/3Wz3D1v-lL8/0.jpg)](https://www.youtube.com/c/MOBATSim)

<a name="requirements"></a>
## **Requirements** 

#### MATLAB version requirement

MOBATSim is continuously updated with the latest version of MATLAB®. Therefore the requirement is MATLAB R2021a or MATLAB R2020b.

#### Toolbox Requirements

* Simulink® and Stateflow®
* Automated Driving Toolbox™
* Robotics System Toolbox™
* Control System Toolbox™
* Model Predictive Control Toolbox™  (only if MPC-Cruise Controller Block is used)
* Simulink 3D Animation Toolbox™ (only required for the 3D Animation Virtual World)

<a name="citation"></a>
## Authors and Contact 

Main Author: Mustafa Saraoglu

Message us via the [contact form](https://mobatsim.com/contact/) on our website!

## Copyright Notice

© 2017 MOBATSim.

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
1. "Fork" the repository from the top right corner.
2. Go to your forked repository and switch to the development branch.
3. Make your changes in the development repository. Make sure your contributions fit the format in terms of coding or input/output properties of the Simulink blocks.
4. To make sure that your changes work, you should run the "MOBATSimAutoTesting.m" and if you get "passed" from all the tests, you should "commit" and then "Push" to your forked repository.
5. Then if you would like to contribute, send a "Pull Request" to the corresponding branch on the MOBATSim/MOBATSim repository.
6. Once it is reviewed, it will be approved or changes will be requested along with the comments of the reviewer regarding the issue with your Pull Request.

We would like to encourage everyone who would like to contribute, so you can also contact us for a more detailed explanation of the structure!

<a name="releasenotes"></a>
## Release Notes - Version 2.0

* New ways to visualize your driving scenario: Unreal Engine support via DrivingScenarioDesigner App, Bird's Eye View.
* New vehicle kinematic bicycle models taken from the Automated Driving Toolbox library.
* A more detailed road structure with actual units as meters and double lane roads.
* New lateral controllers: Stanley lateral controller for common vehicles, Pure Pursuit lateral controller for the Ego Vehicle (Vehicle 2).
* Implementation of Frenet Coordinate system for local trajectory planning.
* Implementation of lane-changing maneuver on double lane roads (at the moment only allowed for Pure Pursuit controller - Ego Vehicle)
* An improved coding structure using superclasses, name-value pairs to also enhance the flexibility of MOBATSim and also code optimization using vectorizations and memory preallocations to increase the performance.
* Detailed documentation for the people who are interested and would like to understand and contribute to MOBATSim
* Bonus content: 3D Animation World with the new Dinosaur park.

## Known Issues and Bugs

* Vehicles are not allowed to start or finish on the intersection points to avoid congestion.
* Bird's Eye View or DrivingScenarioDesigner APP might work slowly because of the size of the road network.
* Some road merges do not have safety guarantees which means that a vehicle just merging another road at the same time with another vehicle or there is a stopping vehicle at the merging point of the joining road may cause collisions.
* Changing the default sample time value of 0.02 or playing with different Simulink Solver options other than "auto" may cause unexpected behavior.

<a name="gettingstarted"></a>
## Getting Started

MOBATSim has a project file that includes the Simulink files and their paths. The project can be opened by double-clicking on **MOBATSim.prj** and a GUI will appear, which can be used to start the simulation. Simply click on "Start Simulation" and wait for the simulation to start.



First it would be best if you [fork the MOBATSim repository](https://github.com/user/repository/fork) and then **clone** it to your computer. After opening the **MOBATSim** folder please refer to the live script file **GettingStarted.mlx** for more detailed documentation.
