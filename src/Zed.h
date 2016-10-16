/*
 * Zed.h
 *
 *  Created on: Sep 12, 2016
 *      Author: Ryan Pope <poperyan73@gmail.com
 */

#ifndef SRC_ZED_H_
#define SRC_ZED_H_

int sgn(double num) {
	return num == 0 ? 0 : (num / fabs(num));
}

float normalize(float joystickValue) {
	if (fabs(joystickValue) <= 0.1) {
		return 0.f;
	}
	return joystickValue;
}


class Zed : public SampleRobot
{
public:
	RobotDrive *drive;
	Joystick *joystick;
	Talon *dropIntakeTalon;
	Talon *intakeTalon;
	Talon *clutchTalon;
	Compressor *compressor;
	DoubleSolenoid *clutchSolenoid;
	DoubleSolenoid *lockSolenoid;
	DigitalInput *limitSwitch;

	std::thread * driveThread;
	std::thread * inputThread;
	bool driveRun = false;

	Zed();
	~Zed();

	void RobotInit();
	void Disabled() {};
	void Autonomous();
	void OperatorControl();
	void Test() {};

	void driveFunc();
	void inputFunc();
};


#endif /* SRC_ZED_H_ */
