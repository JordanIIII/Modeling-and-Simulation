In order for the simulation to work the arm model file needs to be replaced as it refers a path that is currently made on one of the creators' computer.

So in order to make it work:
	1. open G12_FULL_IMPLEMENTATION_PUMA_derivative.emx and take note of the connections to model: robot_arm_puma_derivative
	2. remove the current robot arm model: robot_arm_puma_derivative
	3. open G12_robot_arm_20-sim_file_PUMA_derivative.emx
	4. from this file copy the robot arm model: robot_arm_puma_derivative
	5. go back to G12_FULL_IMPLEMENTATION_PUMA_derivative.emx and paste the model: robot_arm_puma_derivative from the clipboard
	6. Make the same connections as they were in step 1.
   	These are:
		OneJunction1\p2 -> robot_arm_puma_derivative\RotationJointZ1
		OneJunction2\p2 -> robot_arm_puma_derivative\RotationJointY1
		OneJunction3\p2 -> robot_arm_puma_derivative\RotationJointY2
		robot_arm_puma_derivative\P_ee -> ee_pos_sampler\input
		robot_arm_puma_derivative\RotationJointZ1_phi -> Mux1\input1
		robot_arm_puma_derivative\RotationJointY1_phi -> Mux1\input2
		robot_arm_puma_derivative\RotationJointY2_phi -> Mux1\input3

Then the model can be run by pressing the Start Simulator and if everything went correctly it should now show 4 different plots for the end effector position, the torques, the currents, and 3D plot.

There is a chance the 3D plot might not work immediately. If so this might be because it does not grab the correct scenery for the same reason as before. If so, follow the following steps to fix:
	1. In the simulator window double click 3D Animation
	2. In the 3D Properties window that opens under Scenery delete: robot_arm_scenery_file_PUMA_derivative
	3. After the old scenery is correctly deleted. Go to File -> Load Scene and load the following file: G12_robot_arm_scenery_file_PUMA_derivative.scn
	4. Now simply run the simulation again using the "Run Simulation" button (From the start!) and the 3D simulation should work.

As it is right now the model runs with end-effector disturbance and gravity in impedance control + gravity compensation mode. 
The model provides 4 blue blocks which represent places where things can be changed without having to significantly change the model these are:
	1. System Parameters -> here the physical system parameters and control parameters for the impedance, torque and current controllers can be changed.
	2. Trajectory -> here the trajectory can be changed to form the desired reference path.
	3. Control Law -> Here the control law can be changed by for example turning off or on the gravity compensation or impedance control in code in the line tau = transpose(J) * 	F_in + 1*tau_G
	4. Disturbances -> Here disturbances can be added to the system. As the current implementation these disturbances are created by applying a virtual wrench to the end effector 	and calculating the resulting torques on the joints and applying that to the system. These disturbances can be turned off by changing the 1 to a 0 in the is_on gain block

Additionally this folder contains the following files which have been created during the buildup of the project and have been used in the report to show verification and validation steps these files can be ran by opening the file and then pressing Start Simulator and then Run Simulation: 
	G12_ideal_SEA_with_impedance_control.emx
	G12_nonideal_SEA_with_impedance_control.emx


G12_motor_drive_efficiency.emx :
In this file, the motor is subjected to a constant voltage of 48V with a Se element with a resistance (provided by the MR element) at the load that ramps up by a slope of 0.1 and starts at t=20 seconds. The motor "AK10-9 V2.0 KV60" parameters were taken from the [website](https://www.cubemars.com/product/ak10-9-v2-0-kv60-robotic-actuator.html).
Parameters used:
- Phase to Phase Inductance: 181μH
- Winding Resistance*: 0.63Ω
- Kt(Nm/A): 0.198
- Back Drive (couloumb friction): 0.8Nm
- Reduction Ratio: 9:1
- Inertia (gcm²): 1002

For the winding resistance, the manufacturer has provided a phase to phase resistance of 0.195Ω. On using the manufacturer's analysis chart, the angular speed of motor (after gearbox) was determined at a torque of 50Nm, applying the reduction ratio and multiplying it withKt gives us the back emf of the motor which was around 24V. 
The motor is given a constant supply of 48V out of which 24V acts on the resistance of the motor wiring. The current at 50Nm torque is determined from the manufacturer's analysis chart (38A). From this, the resistance of the motor wiring is estimated to be 0.63Ω. 

The plot 'Plot1' shows how Efficiency, Angular Speed, Current drawn varies with Torque (0-60 N.m) and the plot 'Plot2' shows the power output of the motor for the same torque range.

The model can be run by pressing start simulator and it should show the above two plots.

