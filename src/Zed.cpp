/*
 * Zed.cpp
 *
 *  Created on: Sep 12, 2016
 *      Author: Ryan Pope <poperyan73@gmail.com
 */

#include <WPILib.h>
#include <Config.h>
#include <thread>
#include <Zed.h>

Zed::Zed() {
	this->drive = new RobotDrive(DRIVE_FL_TALON, DRIVE_BL_TALON, DRIVE_FR_TALON, DRIVE_BR_TALON);
	this->joystick = new Joystick(JOY_PORT_0);

	this->leftIntakeTalon = new Talon(LEFT_INTAKE_TALON);
	this->rightIntakeTalon = new Talon(RIGHT_INTAKE_TALON);
	this->intakeTalon = new Talon(INTAKE_TALON);
	this->clutchTalon= new Talon(CLUTCH_TALON);

	this->compressor = new Compressor(COMPRESSOR_PORT);

	this->clutchSolenoid = new DoubleSolenoid(CLUTCH_SOL_FORWARD, CLUTCH_SOL_REVERSE);
	this->lockSolenoid = new DoubleSolenoid(LOCK_SOL_FORWARD, LOCK_SOL_REVERSE);

	this->clutchSolenoid->Set(DoubleSolenoid::kReverse);
	this->lockSolenoid->Set(DoubleSolenoid::kReverse);

	this->limitSwitch = new DigitalInput(LIMIT_SWITCH);

	driveThread = new std::thread(driveFunc);
	inputThread = new std::thread(inputFunc);

	driveThread->detach();
	inputThread->detach();
}

Zed::~Zed() {
	delete drive;
	delete joystick;
	delete leftIntakeTalon;
	delete rightIntakeTalon;
	delete intakeTalon;
	delete clutchTalon;
	delete compressor;
	delete clutchSolenoid;
	delete lockSolenoid;
	delete limitSwitch;

	delete driveThread;
	delete inputThread;
}

void Zed::RobotInit() {

}

void Zed::Autonomous() {

}

void Zed::driveFunc() {
	float Kp = 0.044000;
	float Ki = 0.000001;
	float Kd = 0.000001;
	float lPIDError = 0;
	float lPIDIntegral = 0;
	float lCurrentSpeed = 0;
	float rPIDError = 0;
	float rPIDIntegral = 0;
	float rCurrentSpeed = 0;
	double oldTime = GetTime();

	while (driveRun) {
		double currentTime = GetTime();
		// Left wheel PID loop
		// Sets the current error of the speed from the desired speed
		float lCurrentError = normalize(this->joystick->GetRawAxis(JOY_AXIS_LY)) - lCurrentSpeed;
		// Integrate to estimate the past error
		lPIDIntegral += lPIDError * (currentTime - oldTime);
		// Differentiate to estimate future error
		float lPIDDerivative = (lCurrentError - lPIDError) / (currentTime - oldTime);
		// Set the current speed based upon past errors
		lCurrentSpeed += (Kp * lCurrentError) + (Ki * lPIDIntegral) + (Kd * lPIDDerivative);
		// Updates the error for the next iteration
		lPIDError = lCurrentError;

		// Right wheel PID loop
		// Sets the current error of the speed from the desired speed
		float rCurrentError = normalize(this->joystick->GetRawAxis(JOY_AXIS_RY)) - rCurrentSpeed;
		// Integrate to estimate the past error
		rPIDIntegral += rPIDError * (currentTime - oldTime);
		// Differentiate to estimate future error
		float rPIDDerivative = (rCurrentError - rPIDError) / (currentTime - oldTime);
		// Set the current speed based upon past errors
		rCurrentSpeed += (Kp * rCurrentError) + (Ki * rPIDIntegral) + (Kd * rPIDDerivative);
		// Updates the error for the next iteration
		rPIDError = rCurrentError;

		// Update the time for the next iteration
		oldTime = currentTime;

		// Drive the robot using the PID corrected speeds
		drive->TankDrive(lCurrentSpeed, rCurrentSpeed);
	}
}

void Zed::inputFunc() {
	bool winding;
	bool shootReady;

	while (driveRun) {
		// If left bumper is pressed, run intake motors in
		if (this->joystick->GetRawButton(JOY_BTN_LBM)) {
			intakeTalon->Set(1.0f);
		} else {
			intakeTalon->Set(0.f);
		}

		// If right bumper is pressed, run intake motors out
		if (this->joystick->GetRawButton(JOY_BTN_RBM)) {
			intakeTalon->Set(-1.0f);
		} else {
			intakeTalon->Set(0.f);
		}

		// If A button is pressed, drop the intake outside of the frame
		if (this->joystick->GetRawButton(JOY_BTN_A)) {
			rightIntakeTalon->Set(-1.0f);
			leftIntakeTalon->Set(1.0f);
		} else {
			rightIntakeTalon->Set(0.f);
			leftIntakeTalon->Set(0.f);
		}

		// If B button is pressed, lift the intake into the frame
		if (this->joystick->GetRawButton(JOY_BTN_B)) {
			rightIntakeTalon->Set(1.0f);
			leftIntakeTalon->Set(-1.0f);
		} else {
			rightIntakeTalon->Set(0.f);
			leftIntakeTalon->Set(0.f);
		}

		// If left trigger is pressed, and the winch is not wound, start winding it
		if (this->joystick->GetRawButton(JOY_BTN_LTG) && !winding && !shootReady) {
			winding = true;
		}

		// Spin the clutch motor until the limit switch is triggered
		if (winding && !shootReady) {
			this->clutchTalon->Set(1.0f);
			this->clutchSolenoid->Set(DoubleSolenoid::kForward);

			// Stop if limit switch is tripped
			if (!this->limitSwitch->Get()) {
				this->lockSolenoid(DoubleSolenoid::kForward);
				this->clutchTalon->Set(0.f);
				this->clutchSolenoid->Set(DoubleSolenoid::kReverse);
				shootReady = true;
				winding = false;
			}
		}

		// If right trigger is pressed and it is wound, shoot the boulder
		if (this->joystick->GetRawButton(JOY_BTN_RTG) && shootReady) {
			this->lockSolenoid->Set(DoubleSolenoid::kReverse);
		}
	}
}

void Zed::OperatorControl() {
	while (RobotBase::IsEnabled()) {
		driveRun = true;
	}
	driveRun = false;
}

START_ROBOT_CLASS(Zed);
