# Tuning Operations (Tuner Opus)

This folder contains modules to automate PID tuning and system calibration for the robot's self-balancing mechanism.

## Files Overview:

* **`autotune.py` & `tune.py`**: Automated PID tuning scripts that perform step-responses or oscillate the wheels to empirically derive the best Kp, Ki, and Kd values for balance.
* **`autotune_session_*.json`**: Log files containing results, history, and the system response data from automatic tuning runs. 
* **`balance_tuner.ipynb`**: Interactive notebook interface used to run tuning iterations and visually plot the IMU angle overshoot over time.
* **`test_inject.py`**: Script for injecting disturbance profiles (virtual pushes) to analyze the robustness of the balance control loop.