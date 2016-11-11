/*
 * Zed.h
 *
 *  Created on: Sep 12, 2016
 *      Author: Ryan Pope <poperyan73@gmail.com
 */

#include <MMRJoystick.h>

#ifndef SRC_ZED_H_
#define SRC_ZED_H_

int sgn(double num) {
	return num == 0 ? 0 : (num / fabs(num));
}

float normalize(float joystickValue) {
	if (fabs(joystickValue) <= 0.05) {
		return 0.f;
	}
	return -joystickValue;
}


class Zed : public SampleRobot
{
public:
	RobotDrive *drive;
	MMRJoystick *joystick;
	CANTalon *dropIntakeCanTalon;
	CANTalon *intakeCanTalon;
	CANTalon *clutchCanTalon;
	Compressor *compressor;
	DoubleSolenoid *clutchSolenoid;
	DoubleSolenoid *lockSolenoid;

	Zed();
	~Zed();

	void RobotInit();
	void Disabled() {};
	void Autonomous();
	void OperatorControl();
	void Test() {};

};


#endif /* SRC_ZED_H_ */
