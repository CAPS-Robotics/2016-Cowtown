/*
 * MMRJoystick.cpp
 *
 *  Created on: Nov 7, 2016
 *      Author: robotics
 */

#include <MMRJoystick.h>

MMRJoystick::MMRJoystick(uint32_t port, bool isXMode, double deadzone) : Joystick(port) {
	this->deadzone = deadzone;
	this->isXMode = isXMode;
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
	return Joystick::GetRawButton(MMRJoystick::DButtons::JoyBtnA);
}

bool MMRJoystick::IsBPressed() {
	return Joystick::GetRawButton(MMRJoystick::DButtons::JoyBtnB);
}

bool MMRJoystick::IsXPressed() {
	return Joystick::GetRawButton(MMRJoystick::DButtons::JoyBtnX);
}

bool MMRJoystick::IsYPressed() {
	return Joystick::GetRawButton(MMRJoystick::DButtons::JoyBtnY);
}

bool MMRJoystick::IsLeftBumperPressed() {
	return Joystick::GetRawButton(MMRJoystick::DButtons::JoyBtnLBumper);
}

bool MMRJoystick::IsRightBumperPressed() {
	return Joystick::GetRawButton(MMRJoystick::DButtons::JoyBtnRBumper);
}

bool MMRJoystick::IsLeftTriggerPressed() {
	return Joystick::GetRawButton(MMRJoystick::DButtons::JoyBtnLTrigger);
}

bool MMRJoystick::IsRightTriggerPressed() {
	return Joystick::GetRawButton(MMRJoystick::DButtons::JoyBtnRTrigger);
}

bool MMRJoystick::IsBackPressed() {
	return Joystick::GetRawButton(MMRJoystick::DButtons::JoyBtnBack);
}

bool MMRJoystick::IsStartPressed() {
	return Joystick::GetRawButton(MMRJoystick::DButtons::JoyBtnStart);
}

bool MMRJoystick::IsLeftJoystickPressed() {
	return Joystick::GetRawButton(MMRJoystick::DButtons::JoyBtnLStick);
}

bool MMRJoystick::IsRightJoystickPressed() {
	return Joystick::GetRawButton(MMRJoystick::DButtons::JoyBtnRStick);
}

float MMRJoystick::GetLeftX() {
	return Joystick::GetRawAxis(MMRJoystick::DAxes::JoyAxisLX);
}

float MMRJoystick::GetLeftY() {
	return Joystick::GetRawAxis(MMRJoystick::DAxes::JoyAxisLY);
}

float MMRJoystick::GetRightX() {
	return Joystick::GetRawAxis(MMRJoystick::DAxes::JoyAxisRX);
}

float MMRJoystick::GetRightY() {
	return Joystick::GetRawAxis(MMRJoystick::DAxes::JoyAxisRY);
}
