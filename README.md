# Custom Implementation of Quadrotor Ground Effect

[TOC]

# 1 What is it and how to use it?

In this repository, we implemented two ground effect models in AirSim, which can be activated through customized high-level C++/Python APIs. In order to use it, you need to substitute the original AirLib folder with the one provided here and rebuild by running `./build.sh` if Ubuntu is used (check out more on development workflow here: [Windows](https://microsoft.github.io/AirSim/dev_workflow/) or [Linux](For Linux, make code changes in AirLib or Unreal/Plugins folder and then run  to rebuild. This step also copies the build output to Blocks sample project. You can then follow above steps again to re-run.)). However, we are also aware that many researchers have already made their own contributions on the source code, and cannot simply replace their AirLib folder with this one. Therefore, we provide details in the sequel on what mathematical models do we choose for ground effect and how we implement them in the source code. 

# 2. Mathematical models

## 2.1 Quadrotor kinematics and dynamics

The following equations show a standard formulation of quadrotor kinematics and dynamics.
$$
\begin{split}
\dot{p} &= v \\
\dot{R} &= R S(\omega) \\
m \dot{v} &= mg + R f_u + f_a \\
J \dot{\omega} &= J \omega \times \omega + \tau_u + \tau_a
\end{split}
$$

where
$$
\begin{split}
& p \in \mathrm{R}^3: \text{global position} \\
& v \in \mathrm{R}^3: \text{linear velocity} \\
& R \in \mathrm{SO(3)}: \text{attitude rotation matrix} \\
& \omega \in \mathrm{R}^3: \text{body angular velocity} \\
& m, J: \text{mass and inertia matrix} \\
& S(\cdot): \text{skew-symmetric mapping} \\
& g = [0, 0, -9.8]^T: \text{gravity vector} \\
& f_u = [0, 0, T]^T: \text{total thrust from 4 rotors [N]} \\
& \tau_u = [\tau_x, \tau_y, \tau_z]^T: \text{body torque from 4 rotors} \\
& f_a, \tau_a \in \mathrm{R}^3: \text{unknown disturbance forces and torques}
\end{split}
$$

## 2.2 Mixer (mapping of control input to actuator output)

Assumptions:

- All of the rotors face the same direction.
- All of the rotors are the same.

The following equation shows the relation between motor speeds and generalized forces through a mixer matrix.
$$
\eta = 
\begin{bmatrix}
T \\ \tau_\phi \\ \tau_\theta \\ \tau_\psi
\end{bmatrix}
=
\begin{bmatrix}
C_T & C_T & C_T & C_T \\
0 & C_T l_{arm} & 0 & -C_T l_{arm} \\
-C_T l_{arm} & 0 & C_T l_{arm} & 0 \\
-C_Q & C_Q & -C_Q & C_Q
\end{bmatrix}
\begin{bmatrix}
n_1^2 \\ n_2^2 \\ n_3^2 \\ n_4^2
\end{bmatrix} 
= A u
$$

where 
$$
\begin{split}
& C_T: \text{thrust coefficient}=\frac{T}{\rho n^2 D^4} \\
& C_Q: \text{moment/torque coefficient}=\frac{Q}{\rho n^2 D^5} \\
& \rho: \text{air density [kg/m^3]} \\
& n: \text{revolutions per second [1/s]} \\
& D: \text{diameter of propeller [m]} \\
& l_{arm}: \text{distance from motor axis to the center of the drone} \\
& A: \text{actuator effectiveness matrix}
\end{split}
$$
Note: Propeller aerodynamic coefficients used here follow standard definitions as instructed by [UIUC Propeller Data Site](https://m-selig.ae.illinois.edu/props/propDB.html). Moreover, another important  parameter of propeller characteristics is power coefficient which can be formulated as follows:
$$
C_P = \frac{P}{\rho n^3 D^5} = 2\pi C_Q
$$
Ideal thrust is
$$
T_{ideal} = \sum_{i=1}^4 C_T n_i^2
$$
Actual thrust affected by ground effect is
$$
T_{actual} = T_{ideal} f_{GE}
$$
where
$$
f_{GE}: \text{ground effect coefficient}
$$

## 2.3 Ground effect modeling

The following two ground effect models are considered here. Cheeseman-Bennett model is a traditional model for helicopters that reveals simple relation between ground effect coefficient $f_{GE}$ (also, the ratio calculated from the two model) and altitude, whereas Conyers's parametric model is derived from empirical data using test bench. Note there exist also many other multirotor ground effect models in the literature and to the best of our knowledge, there is no widely acknowledged mathematical equation that can be generalized to all possible situations. 

- Cheeseman-Bennett model:

$$
\frac{T_{IGE}}{T_{OGE}} = \frac{1}{1 - (\frac{R}{4Z})^2} \quad \text{iff} \quad \frac{Z}{R}>0.25
$$

​		where $Z$ is the height above the ground, and $R$ is the radius of the propeller.

- Conyers's parametric model:

$$
\frac{T_{IGE}}{T_{OGE}} = \frac{p_1 x^4 + p_2 x^3 + p_3 x^2 + p_4 x + p_5}{x^3 + q_1 x^2 + q_2 x + q_3} \quad \text{where} \quad x = \frac{Z}{R}
$$

​		where $p_1$, $p_2$, $p_3$, $p_4$, $p_5$, $q_1$, $q_2$, $q_3$ are polynomial functions of propeller spacing ($L = 2*l_{arm}$, measured in mm) and propeller radius ($R$, measured in mm), as given in Eq. (6.7) in Conyers's PhD dissertation: *Conyers, Stephen A.. “Empirical Evaluation of Ground, Ceiling, and Wall Effect for Small-Scale Rotorcraft.”  (2019)*.

# 3 Implementation

## 3.1 Background 

We start with providing the background of how thrust and torque (wrench) are calculated in AirSim in the original source code.

- Maximum thrust and torque generated by one single propeller: They are calculated in `RotorParams.hpp`, using thrust and power coefficient.

- Output thrust and torque generated by one single propeller: They are calculated in `RotorActuator.hpp`, based on two parameters:`output.control_signal_filtered` (a value between 0 and 1, can be considered as squared value) and `air_density_ratio_` (forces and torques are proportional to air density). These outputs will be "stored" in `PhysicsBodyVertex.hpp` and read or updated via `getWrench()` and `setWrench()`. The sum of all the forces and torques generated by all the propellers are called "body wrench" in `FastPhysicsEngine.hpp`. But notice that body wrench is ONLY one part of the wrench that the multirotor experiences in the environment!

- Actual total wrench: This eventually applies to the drone taking into account the environmental interaction and is calculated within `updatePhysics()` in `FastPhysicsEngine.hpp`. Specifically, it is the sum of non-collision wrench and collision response. Furthermore, the non-collision wrench is derived by body wrench and drag wrench. 


## 3.2 Ground effect implementation

### 3.2.1 Set up drone geometry

| Data                          | Value    | Source                 | Note                                                         |
| ----------------------------- | -------- | ---------------------- | ------------------------------------------------------------ |
| Thrust coefficient (C_T)      | 0.109919 | `RotorParams.hpp`      | GWS 9x5 propeller @ 6396.667 RPM.                            |
| Power coefficient (C_P)       | 0.040164 | `RotorParams.hpp`      | GWS 9x5 propeller @ 6396.667 RPM.                            |
| Propeller diameter (D)        | 0.2286m  | `RotorParams.hpp`      |                                                              |
| Quadrotor mass (m)            | 1kg      | `MultiRotorParams.hpp` |                                                              |
| Propeller spacing (2*l_{arm}) | 0.690m   | `MultiRotorParams.hpp` | Default value is 0.2275m*2. We modified this number to be aligned with the setup in Conyer’s paper. |

### 3.2.2 Get distance sensor measurements

This is to retrieve altitude above the ground data from distance sensor using AirSim APIs.

Note: Distance sensor is specified in `settings.json` in the directory of `Documents/AirSim` as follows. The position of the sensor is set to be 0.85m below (`"Z: 0.85"`) the center of mass of the multirotor. This value (or bias) is calibrated from simulation, taking the average of subtraction between distance sensor measurement and ground truth when `Z: 0`. 

```
"DistanceCustom": {
	  "SensorType": 5,
	  "Enabled": true,
	  "MinDistance": 0.1, 
	  "MaxDistance": 40,
	  "X": 0, "Y": 0, "Z": 0.85,
	  "Yaw": 0, "Pitch": -90, "Roll": 0,
	  "DrawDebugPoints": true
	},
```

### 3.2.3 Calculate thrust increment/reduction

We assume the ground effect only affects thrust in the vertical axis of the multirotor and there is no influence on either the torques or the forces in horizontal axes. Hence, we have:

- $T_{IGE}$ is derived from Cheeseman-Bennett or Conyers's parametric model.

- $T_{OGE}$ can be set as the weight of quadrotor with known mass or derived using rotor speeds for more precise results.

Therefore, the increment or reduction is calculated by ($T_{IGE}$ - $T_{OGE}$).

Note: Gravity constant takes value using `body.getEnvironment().getState().gravity` from `PhysicsBody` class of AirSim with default value of 9.806650 using default settings (including geo-position etc.).

### 3.2.4 Interact with physics engine

In `MultiRotorPhysicsBody.hpp`:

- Include `#include "MultiRotorGroundEffect.hpp"`
- Create and instantiate a pointer `MultiRotorGroundEffect* multirotor_ge_;` and `multirotor_ge_ = new MultiRotorGroundEffect;`
- Define ground effect function that overrides the virtual one defined in the base class`PhysicsBody.hpp`: `virtual real_T getGroundEffect() const override`

In `FastPhysicsEngine.hpp`:

- Add `getGroundEffect()` into `getNextKinematicsNoCollision()`. Hence the total wrench will be the sum of body wrench, drag wrench, and ground effect, from which the next time step linear acceleration will be derived.

Note: These only provide the idea of how ground effect is implemented into physics engine, and there are some other necessary modifications in the code.

### 3.2.5 Solve numerical stability issues

It is found that the parametric ground effect model might not be generic for all possible pairs of drone geometry, and hence numerical instability might occur. To avoid these issues, the following modifications are applied to the parametric model:

- Output thrust ratio is set (in fact must converge) to 1 when $Z/R > 10$.  
- Output thrust ratio is set to 1 (in fact this could go toward infinite but for numerical stability we choose it to be 1 or say, ignore it) when $Z/R < 0.25$.

### 3.2.6 C++ and Python APIs 

High-level APIs (`simSetGroundEffect()`) for multirotor ground effect are implemented in both C++ and Python to facilitate its use for users. Through these APIs, users can turn on the ground effect and choose specific underlying model (i.e., Cheeseman-Bennett or Conyers's parametric model). The implementation follows [official document](https://microsoft.github.io/AirSim/adding_new_apis.html) with details as follow:

- Step 1: Specify ground effect API as vehicle-based API in `VehicleApiBase.hpp`,

  ```
  virtual void simSetGroundEffect(int model) = 0; 
  ```

  and its implementation in `MavLinkMultirotorApi.hpp`,

  ```
  virtual void setGroundEffect(int model) override
  {
  	...
  }
  ```

  Note that you shall also implement it for other firmwares to avoid compilation errors, i.e., `ArduCopterApi.hpp` for arducopter, `SimpleFlightApi.hpp` for simple_flight, `ArduRoverApi.hpp` for ardurover, and `PhysXCarApi.hpp` for physxcar.

- Step 2: Add an RPC handler in the server which calls your implemented method in`RpcLibServerBase.cpp`. Vehicle-specific APIs are in their respective vehicle subfolder as mentioned in Step 1.

  ```
  pimpl_->server.bind("simSetGroundEffect", [&](int model, const std::string& vehicle_name) -> void {
  	getVehicleApi(vehicle_name)->simSetGroundEffect(model);
  });
  ```

- Step 3: Add the C++ client API method in `RpcLibClientBase.cpp`,

  ```
  void RpcLibClientBase::setGroundEffect(int model, const std::string& vehicle_name)
  {
  	pimpl_->client.call("simSetGroundEffect", model, vehicle_name);
  }
  ```

  and `RpcLibClientBase.hpp`,

  ```
  void setGroundEffect(int model, const std::string& vehicle_name);
  ```

- Step 4: Add the Python client API method in `client.py`,

  ```
  def simSetGroundEffect(self, model, vehicle_name = ''):
          """
          Set ground effect characterised by model for multirotor corresponding to vehicle_name
  
          Args:
              model (int): 0 to no ground effect, 1 to Cheeseman-Bennett model, 2 to Parametric model by Stephen Conyers
              vehicle_name (str, optional): Name of the vehicle to send this command to
          """
          self.client.call('simSetGroundEffect', model, vehicle_name)
  ```

A test script in Python is given as follows to validate the above implementation:

```
###################################################################################
# Description: This script runs the test of multirotor ground effect API in Python.
# Howto: 
#  - Run this script after taking off the multirotor in the simulator.
#  - Press any key to change the underlying ground effect model, you shall see
#    messages printed in both UE4 and python terminals specifying the model 
#    currently under deployment.
# Author: Weibin Gu, Politecnico di Torino @ CSL
# Date: July 29, 2021
###################################################################################

import pprint
import setup_path
import airsim

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

GND_EFFECT_MODEL = 0 # 0: no ground effect; 
                     # 1: Cheeseman-Bennett; 
                     # 2: Conyers's parametric (all specified in AirLib)

while(True):
    airsim.wait_key('Press any key to change ground effect model')
    client.simSetGroundEffect(GND_EFFECT_MODEL)
    print(f"Python API: ground effect with model = {GND_EFFECT_MODEL}")
    GND_EFFECT_MODEL += 1
    if GND_EFFECT_MODEL > 2:
        GND_EFFECT_MODEL = 0
```

# Contact

Should you have any feedback on this implementation, please feel free to reach out to me via: weibin.gu@polito.it

Last Update: June 30, 2021
