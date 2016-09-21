/*
 * config.h
 *
 *  Created on: Sep 12, 2016
 *      Author: Ryan Pope <poperyan73@gmail.com
 */

#ifndef SRC_CONFIG_H_
#define SRC_CONFIG_H_

// Joystick port
#define JOY_PORT_0					0

// Joystick buttons
#define JOY_BTN_X					1
#define JOY_BTN_A					2
#define JOY_BTN_B					3
#define JOY_BTN_Y					4

#define JOY_BTN_LBM					5
#define JOY_BTN_RBM					6
#define JOY_BTN_LTG					7
#define JOY_BTN_RTG					8

#define JOY_SPC_BCK					9  // Back button
#define JOY_SPC_STR					10 // Start button
#define JOY_SPC_LST					11 // Push the left stick in
#define JOY_SPC_RST					12 // Push the right stick in

// Joystick axes
#define JOY_AXIS_LX             	1
#define JOY_AXIS_LY             	2
#define JOY_AXIS_RX             	3
#define JOY_AXIS_RY             	4
#define JOY_AXIS_DX             	5
#define JOY_AXIS_DY             	6

// Talons
#define DRIVE_FL_TALON				0
#define DRIVE_BL_TALON				1
#define DRIVE_FR_TALON				2
#define DRIVE_BR_TALON				3
#define CLUTCH_TALON				4
#define LEFT_INTAKE_TALON			5
#define RIGHT_INTAKE_TALON			6
#define INTAKE_TALON				7

// Solenoids
#define CLUTCH_SOL_FORWARD			0
#define CLUTCH_SOL_REVERSE			1
#define LOCK_SOL_FORWARD			2
#define LOCK_SOL_REVERSE			3

// Compressor
#define COMPRESSOR_PORT				0

// Digital I/O
#define LIMIT_SWITCH				0

#endif /* SRC_CONFIG_H_ */
