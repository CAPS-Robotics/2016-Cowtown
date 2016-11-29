/*
 * MMRDashboard.cpp
 *
 *  Created on: Nov 9, 2016
 *      Author: robotics
 */

#include <MMRDashboard.h>
#include <string>

void MMRDashboard::PutTextboxValue(int id, std::string value) {
	SmartDashboard::PutString("DB/String " + std::to_string(id), value);
}

void MMRDashboard::PutTextboxValue(int id, double value) {
	SmartDashboard::PutString("DB/String " + std::to_string(id), std::to_string(value));
}

void MMRDashboard::PutTextboxValue(int id, int value) {
	SmartDashboard::PutString("DB/String " + std::to_string(id), std::to_string(value));
}

void MMRDashboard::PutSliderValue(int id, float value) {
	SmartDashboard::PutNumber("DB/Slider " + std::to_string(id), value);
}

void MMRDashboard::TurnLedOn(int id) {
	MMRDashboard::PutBoolean("DB/LED " + std::to_string(id), true);
}

void MMRDashboard::TurnLedOff(int id) {
	MMRDashboard::PutBoolean("DB/LED " + std::to_string(id), false);
}

std::string MMRDashboard::GetTextboxValue(int id, std::string def) {
	return MMRDashboard::GetString("DB/String " + std::to_string(id), def);
}

float MMRDashboard::GetSliderValue(int id, float def) {
	return MMRDashboard::GetNumber("DB/Slider " + std::to_string(id), def);
}

bool MMRDashboard::GetLedValue(int id, bool def) {
	return MMRDashboard::GetBoolean("DB/LED " + std::to_string(id), def);
}

void MMRDashboard::ToggleLed(int id) {
	MMRDashboard::PutBoolean("DB/LED " + std::to_string(id), !MMRDashboard::GetLedValue(id, false));
}
