/*
 * MMRDashboard.h
 *
 *  Created on: Nov 9, 2016
 *      Author: robotics
 */


#ifndef SRC_MMRDASHBOARD_H_
#define SRC_MMRDASHBOARD_H_

#include <SmartDashboard/SmartDashboard.h>
#include <string>

class MMRDashboard : public SmartDashboard {
public:
	static void PutTextboxValue(int id, std::string value);
	static void PutTextboxValue(int id, double value);
	static void PutTextboxValue(int id, int value);
	static void PutSliderValue(int id, float value);
	static void TurnLedOn(int id);
	static void TurnLedOff(int id);
	static std::string GetTextboxValue(int id, std::string def = "");
	static float GetSliderValue(int id, float def = 0.0);
	static bool GetLedValue(int id, bool def = false);
	static void ToggleLed(int id);
private:
	MMRDashboard() = default;
	virtual ~MMRDashboard() = default;
};

#endif /* SRC_MMRDASHBOARD_H_ */
