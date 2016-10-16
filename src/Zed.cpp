/*
 * Zed.cpp
 *
 *  Created on: Sep 12, 2016
 *      Author: Ryan Pope <poperyan73@gmail.com
 *
 *  Control Layout:
 *
 *      Left Joystick:  Left wheels
 *      Right Joystick: Right wheels
 *
 *      Left Trigger: 	Begin winding up catapult
 *      Right Trigger: 	Shoot ball once wound up
 *      Left Bumper: 	Intake pick up ball
 *      Right Bumper: 	Intake spit out ball
 *
 *      A Button:		Drop intake to pick up ball
 *      B Button: 		Lift intake back into robot
 */

#include <WPILib.h>
#include <Config.h>
#include <thread>
#include <Zed.h>

Zed::Zed() {
	this->drive = new RobotDrive(DRIVE_FL_TALON, DRIVE_BL_TALON, DRIVE_FR_TALON, DRIVE_BR_TALON);
	this->joystick = new Joystick(JOY_PORT_0);

	this->dropIntakeTalon = new Talon(DROP_INTAKE_TALON);
	this->intakeTalon = new Talon(INTAKE_TALON);
	this->clutchTalon = new Talon(CLUTCH_TALON);

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
	delete dropIntakeTalon;
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
	SmartDashboard::PutString("DB/String 0", "Not Wound");
	SmartDashboard::PutString("DB/String 5", 0);
}

void Zed::Autonomous() {
	int mode = std::stof(SmartDashboard::GetString("DB/String 5", "0"));

	// Basic Auto that drives forward
	if (mode == 0) {
		this->drive->TankDrive(0.75f, 0.75f, false);
		Wait(2.f);
		this->drive->TankDrive(0.f, 0.f, false);
		Wait(1.f);
	}
}

void Zed::driveFunc() {
	float Kp = 0.044000;
	float Ki = 0.000001;
	float Kd = 0.000001;
	float lPIDError 	= 0;
	float lPIDIntegral 	= 0;
	float lCurrentSpeed = 0;
	float rPIDError 	= 0;
	float rPIDIntegral 	= 0;
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
	bool winding = false;
	bool shootReady = false;

	while (driveRun) {
		// If left trigger is pressed, and the winch is not wound, start winding it
		if (this->joystick->GetRawButton(JOY_BTN_LTG) && !winding && !shootReady) {
			winding = true;
		}

		// Spin the clutch motor until the limit switch is triggered
		if (winding && !shootReady) {
			SmartDashboard::PutString("DB/String 0", "Winding");
			this->clutchSolenoid->Set(DoubleSolenoid::kForward);
			this->clutchTalon->Set(0.25f);

			// Stop if limit switch is triggered
			if (!this->limitSwitch->Get()) {
				this->lockSolenoid->Set(DoubleSolenoid::kForward);
				this->clutchTalon->Set(0.f);
				this->clutchSolenoid->Set(DoubleSolenoid::kReverse);
				SmartDashboard::PutString("DB/String 0", "Shooter Ready");
				shootReady = true;
				winding = false;
			}
		}

		// If right trigger is pressed and it is wound, shoot the boulder
		if (this->joystick->GetRawButton(JOY_BTN_RTG) && shootReady && !winding) {
			this->lockSolenoid->Set(DoubleSolenoid::kReverse);
			SmartDashboard::PutString("DB/String 0", "Not Wound");
		}

		// If left bumper is pressed, run intake motors in
		if (this->joystick->GetRawButton(JOY_BTN_LBM)) {
			intakeTalon->Set(0.5f);
		} else {
			intakeTalon->Set(0.f);
		}

		// If right bumper is pressed, run intake motors out
		if (this->joystick->GetRawButton(JOY_BTN_RBM)) {
			intakeTalon->Set(-0.5f);
		} else {
			intakeTalon->Set(0.f);
		}

		// If A button is pressed, drop the intake outside of the frame
		if (this->joystick->GetRawButton(JOY_BTN_A)) {
			dropIntakeTalon->Set(-0.25f);
		} else {
			dropIntakeTalon->Set(0.f);
		}

		// If B button is pressed, lift the intake into the frame
		if (this->joystick->GetRawButton(JOY_BTN_B)) {
			dropIntakeTalon->Set(0.25f);
		} else {
			dropIntakeTalon->Set(0.f);
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
