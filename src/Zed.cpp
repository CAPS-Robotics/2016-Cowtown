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
 *      Y Button:		Stop winding
 */

#include <WPILib.h>
#include <Config.h>
#include <MMRDashboard.h>
#include <thread>
#include <Zed.h>

Zed::Zed() {
	this->drive = new RobotDrive(DRIVE_FL_TALON, DRIVE_BL_TALON, DRIVE_FR_TALON, DRIVE_BR_TALON);
	this->joystick = new MMRJoystick(JOY_PORT_0);
	///this->joystick = new Joystick(JOY_PORT_1);

	this->dropIntakeCanTalon = new CANTalon(DROP_INTAKE_CAN_TALON);
	this->intakeCanTalon = new CANTalon(INTAKE_CAN_TALON);
	this->clutchCanTalon = new CANTalon(CLUTCH_CAN_TALON);


	this->compressor = new Compressor(COMPRESSOR_PORT);

	this->clutchSolenoid = new DoubleSolenoid(CLUTCH_SOL_FORWARD, CLUTCH_SOL_REVERSE);
	this->lockSolenoid = new DoubleSolenoid(LOCK_SOL_FORWARD, LOCK_SOL_REVERSE);

	this->clutchSolenoid->Set(DoubleSolenoid::kReverse);
	this->lockSolenoid->Set(DoubleSolenoid::kReverse);
}

Zed::~Zed() {
	delete drive;
	delete joystick;
	delete dropIntakeCanTalon;
	delete intakeCanTalon;
	delete clutchCanTalon;
	delete compressor;
	delete clutchSolenoid;
	delete lockSolenoid;
}

void Zed::RobotInit() {
	SmartDashboard::PutString("DB/String 0", "Not Wound");
	SmartDashboard::PutString("DB/String 5", "0");
	this->intakeCanTalon->SetPID(0.05f, 0.000096f, 0.8f, 0.f);
	this->intakeCanTalon->SetSensorDirection(false);
	this->intakeCanTalon->SetClosedLoopOutputDirection(false);

	this->dropIntakeCanTalon->SetPID(0.05f, 0.000096f, 0.8f, 0.f);
	this->dropIntakeCanTalon->SetSensorDirection(false);
	this->dropIntakeCanTalon->SetClosedLoopOutputDirection(false);

	this->clutchCanTalon->SetPID(0.05f, 0.000096f, 0.8f, 0.f);
	this->clutchCanTalon->SetSensorDirection(false);
	this->clutchCanTalon->SetClosedLoopOutputDirection(false);

	this->compressor->SetClosedLoopControl(true);

	this->drive->SetSafetyEnabled(false);
}

void Zed::Autonomous() {
	int mode = 1; // std::stoi(SmartDashboard::GetString("DB/String 5", "0"));

	// Basic Auto that drives forward
	if (mode == 0) {
		this->drive->TankDrive(0.75f, 0.75f, false);
		Wait(2.f);
		this->drive->TankDrive(0.f, 0.f, false);
		Wait(1.f);
	} else if (mode == 1) {
		this->compressor->Start();
		while (true) {
			MMRDashboard::PutTextboxValue(1, this->compressor->GetPressureSwitchValue());
			MMRDashboard::PutTextboxValue(2, this->compressor->GetCompressorCurrent());
		}
	} else {

	}
}

void Zed::OperatorControl() {
	float Kp 			= 0.033000;
	float Ki 			= 0.000001;
	float Kd 			= 0.000001;
	float lPIDError 	= 0;
	float lPIDIntegral 	= 0;
	float lCurrentSpeed = 0;
	float rPIDError 	= 0;
	float rPIDIntegral 	= 0;
	float rCurrentSpeed = 0;
	double oldTime = GetTime();
	int multiplier = 1;

	while (RobotBase::IsEnabled()) {
		if (this->joystick->IsStartPressed()) {
			if (multiplier == 1) {
				multiplier = -1;
			} else {
				multiplier = 1;
			}
		}

		double currentTime = GetTime();
		// Left wheel PID loop
		// Sets the current error of the speed from the desired speed
		float lCurrentError = -this->joystick->GetLeftY() - lCurrentSpeed;
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
		float rCurrentError = this->joystick->GetRightY() - rCurrentSpeed;
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
		// Drifts to the right so slow down the right motors
		drive->TankDrive(lCurrentSpeed * SCALE_FACTOR, -rCurrentSpeed * SCALE_FACTOR * 0.9, false);

		if (this->joystick->IsLeftTriggerPressed()) {
			this->clutchCanTalon->Set(multiplier * 0.85f);
		} else {
			this->clutchCanTalon->Set(0.f);
		}

		if (this->joystick->IsXPressed()) {
			this->clutchSolenoid->Set(DoubleSolenoid::kForward);
		}

		if (this->joystick->IsBackPressed()) {
			this->clutchSolenoid->Set(DoubleSolenoid::kReverse);
		}

		if (this->joystick->IsYPressed()) {
			this->lockSolenoid->Set(DoubleSolenoid::kReverse);
			Wait(0.5f);
			this->clutchCanTalon->Set(0.f);
			Wait(0.75f);
			this->clutchSolenoid->Set(DoubleSolenoid::kReverse);
			SmartDashboard::PutString("DB/String 0", "Shooter Ready");
		}

		// Spin the clutch motor until the limit switch is triggered
/*		if (winding && !shootReady) {
			SmartDashboard::PutString("DB/String 0", "Winding");
//			this->clutchSolenoid->Set(DoubleSolenoid::kForward);
			this->clutchCanTalon->Set(0.3f);
			// Stop if Y is pressed
			if (this->joystick->IsYPressed()) {
//				this->lockSolenoid->Set(DoubleSolenoid::kForward);
				this->clutchCanTalon->Set(0.f);
//				this->clutchSolenoid->Set(DoubleSolenoid::kReverse);
				SmartDashboard::PutString("DB/String 0", "Shooter Ready");
				shootReady = true;
				winding = false;
			}
		}*/

		// If right trigger is pressed and it is wound, shoot the boulder
		if (this->joystick->IsRightTriggerPressed()) {
			this->lockSolenoid->Set(DoubleSolenoid::kForward);
			SmartDashboard::PutString("DB/String 0", "Fired");
		}


		// If left bumper is pressed, run intake motors in
		if (this->joystick->IsLeftBumperPressed()) {
			intakeCanTalon->Set(-1.0f);
			Wait(0.02f);
			SmartDashboard::PutString("DB/String 0", "Winding");
		} else {
			intakeCanTalon->Set(0.f);
		}

		// If right bumper is pressed, run intake motors out
		if (this->joystick->IsRightBumperPressed()) {
			intakeCanTalon->Set(1.0f);
			Wait(0.02f);
		} else {
			intakeCanTalon->Set(0.f);
		}

		MMRDashboard::PutTextboxValue(1, dropIntakeCanTalon->Get());

		// If A button is pressed, drop the intake outside of the frame
		if (this->joystick->IsAPressed()) {
			dropIntakeCanTalon->Set(1.f);
			Wait(.25f);
		} else {
			dropIntakeCanTalon->Set(0.f);
		}

		// If B button is pressed, lift the intake into the frame
		if (this->joystick->IsBPressed()) {
			dropIntakeCanTalon->Set(-0.75f);
		} else {
			dropIntakeCanTalon->Set(0.f);
		}
	}
}


START_ROBOT_CLASS(Zed);
