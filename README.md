# **MOBATSim**

MOBATSim (Model-based Autonomous Traffic Simulation Framework) is a simulation framework based on MATLAB Simulink that allows the user to assess vehicle level and traffic level safety by a 3D traffic simulation.

Website: https://mobatsim.com/

[![View MOBATSim_OpenSource on File Exchange](https://www.mathworks.com/matlabcentral/images/matlab-file-exchange.svg)](https://www.mathworks.com/matlabcentral/fileexchange/78444-mobatsim_opensource)

<img src="img/logo_big.jpg" alt="Combined Image" />

- [**MOBATSim**](#--mobatsim--)
  * [Overview](#overview)
  * [Author](#author)
  * [Copyright Notice](#copyright-notice)
- [**Requirements**](#--requirements--)
  * [Matlab version requirement](#matlab-version-requirement)
  * [Toolbox requirements](#toolbox-requirements)

Overview
---
* MOBATSim has a project file which includes the Simulink files and their paths. As you run `MOBATSim.prj`, a GUI will open and you can run simulations from that GUI. The live script is a more detailed documentation than this one.

```
uiopen('\MOBATSim.prj',1)
```

Author
---
Main Author: Mustafa Saraoglu

Copyright Notice
---
© 2017 MOBATSim.

Please cite as:
Saraoglu, M., Morozov, A., & Janschek, K. (2019). MOBATSim: MOdel-Based Autonomous Traffic Simulation Framework for Fault-Error-Failure Chain Analysis. IFAC-PapersOnLine, 52(8), 239–244. Elsevier BV. Retrieved from https://doi.org/10.1016%2Fj.ifacol.2019.08.077

# **Requirements**

MATLAB version requirement
---
MOBATSim is continously updated with different versions of MATLAB. Therefore, at this moment, the requirement is MATLAB 2020a.

Toolbox requirements
---
* Simulink 3D Animation Toolbox™ (required for the 3D Animation Model)
* Model Predictive Control Toolbox™  (will be soon required)
* Simulink Coverage™ (will be required in the next versions for testing and code coverage analyses)
