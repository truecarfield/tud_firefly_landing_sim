tud_firefly_landing_sim is a ROS package based on rotors_simulato. In the package, series of control and state estimation methods has been designed or integrated to simulate the process of autonomous landing control of a hexacopter UAV (Unmanned Aerial Vehicle) AscTec firefly on top of a mobile UGV (Unmanned Ground Vehicle) using onboard vision. The package is created modularly as a metapackage so that other ones could also test their own algorithms using this package.

In order to compile this package correctly, several other packages are required:
ar_sys: http://wiki.ros.org/ar_sys
rotors_simulator: https://github.com/ethz-asl/rotors_simulator
robot_localization: http://wiki.ros.org/robot_localization
joystick_drivers(no dependence, only used with launch scripts): http://wiki.ros.org/joystick_drivers


The pdf version of the tutorial of tud_firefly_landing_sim is available in  
tud_firefly_landing_sim_tutorial.pdf

Overall there are seven subpackages in this metapackage:
1. tud_firefly_landing_sim: This package plays the role of package list of a metapackage, in the
 package.xml file of this package all subpackages are listed in the run_depend list;

2. tud_firefly_description: This package contains the customerized marker mesh files and the URDF
 scripts of the AscTec firefly UAV as well as the UGV that to be landed;

3. tud_firefly_gazebo_plugins: This package contains source files of the gazebo plugins from the
 UAV model such as actuator controller, sensors like IMU and camera;

4. tud_firefly_state_estimation: This package contains the data fusion system of the UAV. Firstly
 an EKF algorithm developed based on the package robot_localization. Moreover, another algorithm called mahony-complementary-filter is developed in order to estimate the UAV attitude
directly from IMU measurements, the final fused UAV states will be published to the topic
 /odometry/filtered
 ;

5. tud_firefly_control: This package contains the source code of UAV control algorithms such as
 attitude controller, twist controller, landing controller etc.;

6. tud_firefly_application: This package contains the source code of several subsystems that contribute to the UAV autonomous landing simulation, such as the marker detection system based on ar_sys. And a landing state machine has been developed in order to define the landing process
 according to current measurements;

7. tud_firefly_gazebo: This package contains the launch files of the UAV autonomous landing simulation.



