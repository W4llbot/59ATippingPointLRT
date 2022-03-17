#include "main.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	Motor FL(FLPort, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
	Motor BLU(BLUPort, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
	Motor BLD(BLDPort, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
	Motor FR(FRPort, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);
	Motor BRU(BRUPort, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);
	Motor BRD(BRDPort, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);

	Motor arm(armPort, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
	Motor intake(intakePort, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);

	// Pneumatic init
	ADIDigitalOut tilt(tiltPort);
	ADIDigitalOut tiltClamp(tiltClampPort);
	ADIDigitalOut armClamp(armClampPort);

	// Sensor Init
  	ADIDigitalIn armLimit(armLimitPort);
	ADIDigitalIn tiltLimit(tiltLimitPort);
	ADIAnalogIn potentiometer(potentiometerPort);

	// Mech tasks
	Task armControlTask(armControl, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Arm Control Task");
	Task tiltControlTask(tiltControl, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Tilt Control Task");
	Task intakeControlTask(intakeControl, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Intake Control Task");

	Task sensorTask(sensors, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Sensor Task");
	//Task debugTask(Debug, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Debug Task");


	//temp enable odom/pp task
	// Task odometryTask(Odometry, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Odom Task");
	//Task controlTask(PPControl, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "PP Task");

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	// drive(50, 50);
	double start = millis();
	// setArmPos(2);
	setOffset(-133.5);
	baseTurn(-133.5);
	delay(100);
	Task odometryTask(Odometry, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Odom Task");
	Task controlTask(PPControl, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "PP Task");

	setMaxRPMV(530);
	baseMove(-26);
	waitPP(2000);

	delay(300);

	enableBase(true, true);
	baseTurn(calcBaseTurn(2, 58, false));
	waitTurn(2000);
	setIntake(100);

	setArmClampState(false);
	baseMove(75);
	waitPP(300);

	setArmClampState(false);
	delay(100);
	baseTurn(calcBaseTurn(-37, 60, false));
	waitTurn(2000);

	setTiltState(false);
	delay(500);
	setArmClampState(false);
	baseMove(70);
	waitPP(3000);

	setArmClampState(false);
	delay(100);
	baseTurn(calcBaseTurn(-72, 60, false));
	waitTurn(2000);

	double smooth = 0.75;
	setArmClampState(false);
	basePP({position, Node(-72, 60)}, 1-smooth, smooth, 14);
	waitPP(3000);
	setArmClampState(true);

	basePP({position, Node(-72, 110)}, 1-smooth, smooth, 14);
	waitPP(2000);


	//-33 60
	//dis 90
	// -70 60


  // double start = millis();
	// setArmPos(2);
	// setOffset(-76);
	// baseTurn(-76);
	// delay(100);
	// Task odometryTask(Odometry, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Odom Task");
	// Task controlTask(PPControl, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "PP Task");
	//
	// setMaxRPMV(530);
	// baseMove(-5);
	// waitPP(700);
	//
	// delay(500);
	// setIntake(95);
	//
  // setArmClampState(false);
	// baseMove(5);
	// waitPP(700);
	// setArmPos(0);
	//
	// delay(300);
	// // setIntake(0);
	//
	// enableBase(true, false);
	// baseTurn(82);
	// delay(500);
	// setIntake(0);
	// waitTurn(1000);
	// setTiltState(false);
	// enableBase(true, true);
	//
	// // 90,19
	// std::vector<Node> moveTowardsGoal = {position, Node(80, 19)};
	// double smooth = 0.75;
	// basePP(moveTowardsGoal, 1-smooth, smooth, 16);
	// waitPP(2000);
	//
	// baseTurn(calcBaseTurn(97, 18, true), 0.105);
  // waitTurn(1000);
	//
	// setArmPos(1);
	// basePP({position, Node(97, 19)}, 1-smooth, smooth, 14, true);
	// waitPP(2000);
	//
	// baseTurn(calcBaseTurn(85, 56, false), 0.14);
	// waitTurn(1000);
	// setIntake(100);
	// setArmClampState(false);
	// setArmPos(0);
	//
	// basePP({position, Node(80, 55.5)}, 1-smooth, smooth, 14);
	// waitPP(2000);
	//
	// basePP({position, Node(90, 5)}, 1-smooth, smooth, 14, true);
	// waitPP(2000);


	// enableBase(true, false);
	// baseTurn(calcBaseTurn(position.getX(), position.getY() + 20, false));
	// waitTurn(1000);
	// setArmPos(1);
	// enableBase(true, true);
	//
	// setIntake(100);
	// baseMove(24);
	// waitPP(1000);
	//
	// setArmPos(0);
	// baseTurn(calcBaseTurn(position.getX()-20, position.getY() + 20, false));
	// waitTurn(1000);
	//
	// setArmClampState(false);
	// baseMove(10);
	// waitPP(1000);
	//
	// basePP({position, Node(90, 5)}, 1-smooth, smooth, 14, true);
	// waitPP(2000);

// printf("\n auton ended in %.2f seconds\n", millis() - start);

}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	Motor FL(FLPort);
	Motor BLU(BLUPort);
	Motor BLD(BLDPort);
	Motor FR(FRPort);
	Motor BRU(BRUPort);
	Motor BRD(BRDPort);
	Motor arm(armPort, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
	Motor intake(intakePort, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);

	ADIDigitalOut tilt(tiltPort);
	ADIDigitalOut tiltClamp(tiltClampPort);
	ADIDigitalOut armClamp(armClampPort);

	FL.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	BLU.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	BLD.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	FR.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	BRU.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	BRD.set_brake_mode(E_MOTOR_BRAKE_BRAKE);

	Controller master(E_CONTROLLER_MASTER);

	int armPos = 0;
	bool tankDrive = true;
	double intakeSpeed = 110;
	while(true) {
		double left, right;
		if(master.get_digital_new_press(DIGITAL_Y)) tankDrive = !tankDrive;

		if(tankDrive) {
			left = master.get_analog(ANALOG_LEFT_Y);
			right = master.get_analog(ANALOG_RIGHT_Y);
		} else {
			double power = master.get_analog(ANALOG_LEFT_Y);
			double turn = master.get_analog(ANALOG_RIGHT_X);
			left = power + turn;
			right = power - turn;
		}

		FL.move(left);
		BLU.move(left);
		BLD.move(left);
		FR.move(right);
		BRU.move(right);
		BRD.move(right);

		if(master.get_digital_new_press(DIGITAL_L1) && armPos < 2) driverArmPos(++armPos);
		else if(master.get_digital_new_press(DIGITAL_L2) && armPos > 0) driverArmPos(--armPos);

		if(master.get_digital_new_press(DIGITAL_R2)) toggleArmClampState();

		if(master.get_digital_new_press(DIGITAL_X)) toggleTiltState();

		setIntake(master.get_digital(DIGITAL_R1)*intakeSpeed);

		posPrintMaster();

		delay(5);
  	}
}
