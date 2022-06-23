# ulogPX4QuadcopterAnimation
Matlab files that can be used to animate the flight trajectory of a quadcopter using a .ulog PX4 log file. Also include files to plot RCAC gains and autotuned parameters [1].

## Files
* **animate_Test.m** : Animates quadcopter trajectory using PX4 log file (ulog format).
* **animate_Test_Theta.m** : nimates quadcopter trajectory using PX4 log file (ulog format). Also, shows evolution of RCAC theta parameters [1].
* **animate_Test_Theta_Autotuned.m** : Animates quadcopter trajectory using PX4 log file (ulog format). Also, shows autotuned RCAC theta parameters [1].

## References

* Quadcopter animation files modified from [https://github.com/gibiansky/experiments/tree/master/quadcopter/matlab](https://github.com/gibiansky/experiments/tree/master/quadcopter/matlab)
* 3D arrow program obtained from [https://www.mathworks.com/matlabcentral/fileexchange/71994-arrow-3d](https://www.mathworks.com/matlabcentral/fileexchange/71994-arrow-3d)

## Citing work

* **[1] Spencer, J., Lee, J., Paredes, J. A., Goel, A., & Bernstein, D.**. An adaptive PID autotuner for multicopters with experimental results. arXiv preprint arXiv:2109.12797, accepted to ICRA 2022
