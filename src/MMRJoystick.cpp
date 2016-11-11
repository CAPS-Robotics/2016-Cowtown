/*
 * MMRJoystick.cpp
 *
 *  Created on: Nov 7, 2016
 *      Author: robotics
 */

#include <MMRJoystick.h>

MMRJoystick::MMRJoystick(uint32_t port, double deadzone) : Joystick(port) {
	this->deadzone = deadzone;
}

MMRJoystick::~MMRJoystick() {

}

float MMRJoystick::Normalize(float value) {
	if (fabs(value) < this->deadzone) {
		return 0;
	}
	return deadzone;
}

double MMRJoystick::GetDeadzone() {
	return this->deadzone;
}

bool MMRJoystick::IsAPressed() {
	return Joystick::GetRawButton(MMRJoystick::Buttons::JoyBtnA);
}

bool MMRJoystick::IsBPressed() {
	return Joystick::GetRawButton(MMRJoystick::Buttons::JoyBtnB);
}

bool MMRJoystick::IsXPressed() {
	return Joystick::GetRawButton(MMRJoystick::Buttons::JoyBtnX);
}

bool MMRJoystick::IsYPressed() {
	return Joystick::GetRawButton(MMRJoystick::Buttons::JoyBtnY);
}

bool MMRJoystick::IsLeftBumperPressed() {
	return Joystick::GetRawButton(MMRJoystick::Buttons::JoyBtnLBumper);
}

bool MMRJoystick::IsRightBumperPressed() {
	return Joystick::GetRawButton(MMRJoystick::Buttons::JoyBtnRBumper);
}

bool MMRJoystick::IsLeftTriggerPressed() {
	return Joystick::GetRawButton(MMRJoystick::Buttons::JoyBtnLTrigger);
}

bool MMRJoystick::IsRightTriggerPressed() {
	return Joystick::GetRawButton(MMRJoystick::Buttons::JoyBtnRTrigger);
}

bool MMRJoystick::IsBackPressed() {
	return Joystick::GetRawButton(MMRJoystick::Buttons::JoyBtnBack);
}

bool MMRJoystick::IsStartPressed() {
	return Joystick::GetRawButton(MMRJoystick::Buttons::JoyBtnStart);
}

bool MMRJoystick::IsLeftJoystickPressed() {
	return Joystick::GetRawButton(MMRJoystick::Buttons::JoyBtnLStick);
}

bool MMRJoystick::IsRightJoystickPressed() {
	return Joystick::GetRawButton(MMRJoystick::Buttons::JoyBtnRStick);
}

float MMRJoystick::GetLeftX() {
	return Joystick::GetRawAxis(MMRJoystick::Axes::JoyAxisLX);
}

float MMRJoystick::GetLeftY() {
	return Joystick::GetRawAxis(MMRJoystick::Axes::JoyAxisLY);
}

float MMRJoystick::GetRightX() {
	return Joystick::GetRawAxis(MMRJoystick::Axes::JoyAxisRX);
}

float MMRJoystick::GetRightY() {
	return Joystick::GetRawAxis(MMRJoystick::Axes::JoyAxisRY);
}
