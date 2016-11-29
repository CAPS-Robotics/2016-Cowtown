/*
 * MMRJoystick.h
 *
 *  Created on: Nov 7, 2016
 *      Author: robotics
 */

#ifndef SRC_MMRJOYSTICK_H_
#define SRC_MMRJOYSTICK_H_

#include <Joystick.h>
#include <cstdint>

class MMRJoystick : public Joystick {
private:
	double deadzone;
	float Normalize(float value);
	bool isXMode;

public:
	MMRJoystick(uint32_t port, bool isXMode = false, double deadzone = 0.05);
	virtual ~MMRJoystick();

	double GetDeadzone();
	void SetDeadzone();

	bool IsAPressed();
	bool IsBPressed();
	bool IsXPressed();
	bool IsYPressed();
	bool IsLeftBumperPressed();
	bool IsRightBumperPressed();
	bool IsLeftTriggerPressed();
	bool IsRightTriggerPressed();
	bool IsBackPressed();
	bool IsStartPressed();
	bool IsLeftJoystickPressed();
	bool IsRightJoystickPressed();

	float GetLeftX();
	float GetLeftY();
	float GetRightX();
	float GetRightY();

	enum DButtons : uint32_t {
		JoyBtnX = 1,
		JoyBtnA,
		JoyBtnB,
		JoyBtnY,
		JoyBtnLBumper,
		JoyBtnRBumper,
		JoyBtnLTrigger,
		JoyBtnRTrigger,
		JoyBtnBack,
		JoyBtnStart,
		JoyBtnLStick,
		JoyBtnRStick
	};

//	enum class XButtons : uint32_t {
//		JoyBtnX = 1,
//		JoyBtnA,
//		JoyBtnB,
//		JoyBtnY,
//		JoyBtnLBumper,
//		JoyBtnRBumper,
//		JoyBtnBack,
//		JoyBtnStart,
//		JoyBtnLStick,
//		JoyBtnRStick
//	};

	enum DAxes : uint32_t {
		JoyAxisLX = 0,
		JoyAxisLY,
		JoyAxisRX,
		JoyAxisRY
	};

//	enum class XAxes : uint32_t {
//		JoyAxisLX = 0,
//		JoyAxisLY,
//		JoyAxisLTrigger,
//		JoyAxisRTrigger,
//		JoyAxisRX,
//		JoyAxisRY
//	};
};

#endif /* SRC_MMRJOYSTICK_H_ */
