## HOW TO RUN

```shell
./install_tools.sh
catkin_make -j1
source devel/setup.zsh
roslaunch mpc_car simulation.launch
```
## HOW TO TURN PARAMETERS

```shell
./src/mpc_car/config/mpc_car.yaml -> mpc parameters
./src/car_simulator/config/car_simulator.yaml -> initial states (in simulation)
```

## HOW TO MODIFY CODES

> Implement MPC of tracking the reference trajectory in C++;

```shell
min  J = \sum_{i=0}^N (x-x_r)^2+ (y-y_r)^2 + rho * (phi-phi_r)^2

s.t. -0.1 <= v_k <= v_max
     |a_k| <= a_max
     |delta_k| <= delta_max
     |delta_{k+1} - delta_k| / dt <= ddelta_max
```

__[ATTENTION]__ Only <TODO: > of codes in src/mpc_car/include/mpc_car/mpc_car.hpp is required.

## AN EXAMPLE

<p align="center">
    <img src="mpc.gif" width="400"/>
</p>
# L6Homework_4.25
