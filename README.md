# **MOBATSim**

MOBATSim (Model-based Autonomous Traffic Simulation Framework) is a framework based on MATLAB® and Simulink® that allows users to develop automated driving algorithms and assess their safety and performance. By running a traffic simulation, the safety of the implemented component or algorithm can be measured on both the vehicle level and the traffic level, supported by 2D and 3D visualization options.

[![MOBATSim - MOBATSim](https://img.shields.io/static/v1?label=MOBATSim&message=MOBATSim&color=96D1AA&logo=github)](https://github.com/MOBATSim/MOBATSim)
[![GitHub tag](https://img.shields.io/github/tag/MOBATSim/MOBATSim?include_prereleases=&sort=semver&color=96D1AA)](https://github.com/MOBATSim/MOBATSim/releases/)
[![Cite our paper](https://img.shields.io/badge/Cite%20our%20paper-BibTex-blueviolet)](#citation)


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

MOBATSim (Model-based Autonomous Traffic Simulation Framework) is a tool developed in MATLAB® and Simulink® for educational purposes, where interested users can implement and simulate automated control and decision-making algorithms in an urban traffic environment.
If you would like to read more about MOBATSim and how we use it for simulation-based testing, make sure you check our [scientific paper](https://www.sciencedirect.com/science/article/pii/S2405896319304100).

You can also visit our [YouTube Channel](https://www.youtube.com/c/MOBATSim)!

[![YouTube Channel](http://img.youtube.com/vi/EjUDVksA7gA/0.jpg)](https://www.youtube.com/c/MOBATSim)

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

MOBATSim's current version is developed and updated with MATLAB and Simulink, release 2020b. Therefore the requirement to run MOBATSim is at least **MATLAB R2020b** or higher. Please let us know if you have issues running MOBATSim. Some of the components and systems require the following toolboxes:

* Simulink®
* Automated Driving Toolbox™
* Control System Toolbox™
* Deep Learning Toolbox™
* Model Predictive Control Toolbox™
* Robotics System Toolbox™
* Simulink 3D Animation Toolbox™ (only required for the 3D Animation Virtual World)
* Stateflow®
* Symbolic Math Toolbox™


<a name="citation"></a>

### If you use MOBATSim for scientific work please cite our related paper as:

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

**MOBATSim's Contributors:** Sheng Ding, Manuel Schirmer, Johannes Pintscher, Laura Slabon, Qianwei Yang, Qihang Shi, Wenkai Wu, Maoxuan Zhao, Erik Noack, Fabian Hart, Müjdat Korkmaz, Marta Valdes Martin, Mustafa Saraoğlu

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

* New ways to visualize your driving scenario: **Unreal Engine 4** support via **Driving Scenario Designer App** and **Bird's-Eye Scope** options added.
* New vehicle **kinematic bicycle models**.
* A more detailed road structure with actual units as meters and double lane roads.
* New lateral controllers are easily chosen using a variant subsystem: **Stanley lateral controller** and **Pure Pursuit lateral controller** with lane-changing capabilities.
* Implementation of the **Frenet Coordinate system** for local trajectory planning and optimal trajectory generation for maneuvers.
* Implementation of **lane-changing maneuver** on double lane roads.
* An improved coding structure using superclasses, name-value pairs to also enhance the flexibility of MOBATSim and also code optimization using vectorizations and memory preallocations to increase the performance.
* Detailed documentation for the people who are interested and would like to understand and contribute to MOBATSim
* Bonus content: **3D Animation World** with new dinosaurs.

## Known Issues and Bugs

* Vehicles are not allowed to choose any node inside or around the intersection as starting or destination points.
* **Bird's-Eye Scope** or **Driving Scenario Designer App** might work slowly because of the size of the road network.
* Some road merges do not have safety guarantees which means that collisions may happen if two vehicles join at the same time.
* Changing the default sample time value of `0.02` or playing with different **Simulink Solver** options other than `auto` may cause unexpected behavior.

<a name="gettingstarted"></a>
## Getting Started

MOBATSim has a project file that includes the Simulink files and their paths. The project can be opened by double-clicking on `MOBATSim.prj` and a GUI will appear, which can be used to start the simulation. Simply click on `Start Simulation` and wait for the simulation to start.
After opening the **MOBATSim** folder please refer to the live script file `GettingStarted.mlx` for more detailed documentation.
