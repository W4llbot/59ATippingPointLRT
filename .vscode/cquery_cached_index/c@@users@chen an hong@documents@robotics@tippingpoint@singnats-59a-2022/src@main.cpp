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
	//Task odometryTask(Odometry, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Odom Task");
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
	setOffset(-79.5);
	baseTurn(-79.5);
	delay(100);
	Task odometryTask(Odometry, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Odom Task");
	Task controlTask(PPControl, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "PP Task");
	std::vector<Node> testPath = {Node(0, 0), Node(0, 72)};
	std::vector<Node> moveTurnPath = {Node(0, 0), Node(0, 24), Node(48, 48)};
	std::vector<Node> reverseMoveTurnPath = {Node(48, 48), Node(24, 24), Node(0, 0)};
	std::vector<Node> straightPath = {Node(0, 0), Node(0, -24), Node(24, -36)};

	setMaxRPMV(530);
	baseMove(-5);
	waitPP(700);

	delay(200);

  setArmClampState(false);
	baseMove(5);
	waitPP(700);

	enableBase(true, false);
	baseTurn(22);
	waitTurn(1000);


	// setMaxRPMV(500);
	std::vector<Node> initEdgeTurn = {position, Node(14, 58)};
	double 		smooth = 0.75;
	basePP(initEdgeTurn, 1-smooth, smooth, 20);

	// waitArmClamp(15000);
	// delay(50);
	waitPP(2000);
	setArmPos(2);
	setIntake(127);

	// std::vector<Node> moveToPlatform1 = {position, Node(40, 93)};
	// basePP(moveToPlatform1, 1-smooth, smooth, 12);
	// waitPP(2000);
	// setCurvK(0.00000000000000012);
	// setMaxRPMV(300);
	std::vector<Node> moveToRings1 = {position, Node(53, 81)}; //, Node(23, 76)
	basePP(moveToRings1, 1-smooth, smooth, 14);
	waitPP(2000);
	// setCurvK(0.0000000000000002);

	// delay(1000);

  enableBase(true, true);
  baseMove(-5);
  waitPP(700);

	baseTurn(-6);
	waitTurn(1000);

  delay(200);

	baseMove(15);
	waitPP(1000);

	setArmPos(1);
	delay(500);
	setArmClampState(false);
	delay(500);

  printf("\n1 goal in %.2f\n", millis() - start);

	baseTurn(0, 0.17);
	waitTurn(0);

  baseMove(-15);
  waitPP(2000);

  setArmPos(0);
  // baseTurn(50, 57, 0.14, false);
  baseTurn(calcBaseTurn(48, 57, false), 0.13);
  waitTurn(2000);
  setTiltState(false);
  delay(800);


	// setCurvK(0.0000000000000002);
	setArmClampState(false);
	baseMove(50, 57, false);
	waitPP(2000);
	delay(200);

	setMaxRPMV(400);
	setArmHeight(1200);
	setIntake(0);
  std::vector<Node> disposeGoal = {position, Node(71, 25)};
	basePP(disposeGoal, 1-smooth, smooth, 12);
	waitPP(2000);

	// setCurvK(0.0000000000000002);

	printf("\ngoal disposed in %.2f\n", millis() - start);

	// delay(200);
  // baseTurn(-90);
	setArmHeight(1100);
	baseTurn(calcBaseTurn(106, 24, true), 0.12);
  waitTurn(1000);
	delay(300);
	setArmPos(0);
  setArmClampState(false);
  delay(200);
	setMaxRPMV(530);

  setTiltState(false);
	setMaxRPMV(300);
  baseMove(99, 24, true);
  waitPP(2000);
	setMaxRPMV(530);
	setIntake(127);
  // basePP(2000);

	baseMove(82, 22, false);
	waitPP(1000);

	baseTurn(calcBaseTurn(82, 56, false), 0.145);
	waitTurn(1000);

	setArmClampState(false);
	setIntake(127);
	baseMove(82, 56, false);
	waitPP(2000);

	setArmPos(2);
  std::vector<Node> moveToGoal = {position, Node(77, 72), Node(64, 100)};
	basePP(moveToGoal, 1-smooth, smooth, 8);
	waitPP(3000);

	// baseTurn(-7);
	// waitTurn(1000);
	setArmPos(1);
	delay(300);
	setArmClampState(false);
	delay(300);

	printf("\n2 goals in %.2f\n", millis() - start);

	baseMove(-5);
	waitPP(1000);

	setArmPos(0);
	baseTurn(calcBaseTurn(36, 95, false), 0.14);
	waitTurn(2000);

	setArmClampState(false);
	baseMove(47, 95, false);
	waitPP(3000);
	setArmClampState(true);
	setArmPos(2);
	setTiltState(false);
	delay(600);
	// delay(300);

	enableBase(true, false);
	baseTurn(calcBaseTurn(38, 110, false));
	// baseTurn(0);
	waitTurn(2000);

	// delay(200);
	//
	// baseMove(36, 98, false);
	// waitPP(1000);

	setArmPos(1);
	delay(300);
	setArmClampState(false);
	delay(300);
	printf("\n3 goals in %.2f\n", millis() - start);

	// baseMove(-8);
	// waitPP(1000);

	enableBase(true, true);
	baseTurn(calcBaseTurn(-12, 97, true), 0.13);
	// baseTurn(calcBaseTurn(50, position.getY(), true));
	waitTurn(2000);

	setTiltState(false);
	setArmPos(0);
	basePP({position, Node(-1, 97)}, 1-smooth, smooth, 10, true);
	// baseMove(0, 92, true);
	waitPP(2000);

	enableBase(true, true);
	baseTurn(calcBaseTurn(60, 91, false), 0.16);
	waitTurn(2000);

	// setCurvK(0.000000000000000171);
					 // 0.000000000000000171

	setArmClampState(false);
	// baseMove(71, 91, false);
	basePP({position, Node(60, 91)}, 1-smooth, smooth, 14);
	waitPP(3000);

	// setCurvK(0.0000000000000002);

	setArmPos(2);
	delay(500);
	// basePP({position, Node(55, 91)}, 1-smooth, smooth, 14, true);
	// waitPP(2000);

	enableBase(true, true);
	baseTurn(calcBaseTurn(60, 112, false), 0.115);
	waitTurn(2000);

	setArmPos(1);
	delay(300);
	baseMove(58, 97, false);
	waitPP(1000);

	setArmPos(1);
	delay(100);
	setArmClampState(false);
	delay(300);
	printf("\n4 goals in %.2f\n", millis() - start);

	baseMove(60, 90, true);
	waitPP(1000);

	setArmPos(0);
	baseTurn(calcBaseTurn(105, 92, false));
	waitTurn(2000);

	// baseTurn(calcBaseTurn(92, 95, false));
	// waitTurn(2000);

	baseMove(105, 92, false);
	waitPP(2000);

	baseTurn(calcBaseTurn(85, 110, false));
	waitTurn(2000);

	setMaxRPMV(300);
	setArmClampState(false);
	baseMove(86, 110, false);
	waitPP(2000);
	setMaxRPMV(530);
	setArmClampState(true);

	setArmHeight(1200);
	baseMove(96, 93, true);
	waitPP(2000);

	baseTurn(calcBaseTurn(95, 3, false), 0.12);
	waitTurn(2000);

	setArmPos(2);
	// baseMove(98, 0, false);
	basePP({position, Node(95, 3)}, 1-smooth, smooth, 14);
	waitPP(4000);

	enableBase(true, false);
	baseTurn(calcBaseTurn(0, position.getY()+3, false));
	waitTurn(2000);

	setArmPos(0);
	delay(500);

	controlTask.suspend();
	delay(50);
	park(70);
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
	double intakeSpeed = 127;
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

		delay(5);
  	}
}
