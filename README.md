# ulogPX4QuadcopterAnimation
Matlab files that can be used to animate the flight trajectory of a quadcopter using a .ulog PX4 log file. Also include files to plot RCAC gains and autotuned parameters [1].

## Files
* **animate_Test.m** : Animates quadcopter trajectory using PX4 log file (ulog format).
* **animate_Test_Theta.m** : Animates quadcopter trajectory using PX4 log file (ulog format). Also, shows evolution of RCAC theta parameters [1].
* **animate_Test_Theta_Autotuned.m** : Animates quadcopter trajectory using PX4 log file (ulog format). Also, shows autotuned RCAC theta parameters [1].

## References

* Quadcopter animation files modified from [https://github.com/gibiansky/experiments/tree/master/quadcopter/matlab](https://github.com/gibiansky/experiments/tree/master/quadcopter/matlab)
* 3D arrow program obtained from [https://www.mathworks.com/matlabcentral/fileexchange/71994-arrow-3d](https://www.mathworks.com/matlabcentral/fileexchange/71994-arrow-3d)

## Citing work

* **[1] Spencer, J., Lee, J., Paredes, J. A., Goel, A., & Bernstein, D.** "An Adaptive PID Autotuner for Multicopters with Experimental Results," 2022 International Conference on Robotics and Automation (ICRA), Philadelphia, PA, USA, 2022, pp. 7846-7853, doi: 10.1109/ICRA46639.2022.9812065.
