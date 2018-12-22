/*
 * PlotterController.h
 *
 *  Created on: Oct 14, 2018
 *      Author: Martin
 */

#ifndef PLOTTERCONTROLLER_H_
#define PLOTTERCONTROLLER_H_

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

#include "DigitalIoPin.h"


#define X_STEP_PIN 		24
#define Y_STEP_PIN 		27

#define X_DIR_PORT		1
#define X_DIR_PIN		0
#define Y_DIR_PORT		0
#define Y_DIR_PIN		28

#define PEN_PIN			10
#define LASER_PIN		12

#define STEPS_PER_MM	88			// 1 for the emulator, 88 for the plotter

#define SCT_INTERRUPT_PRIORITY				5
#define LIMIT_SWITCH_INTERRUPT_PRIORITY		4


#define POSITIVE_XY_DIRECTION				1		// Direction pin value that moves the motors in the increasing coordinate direction (from min to max)
#define NEGATIVE_XY_DIRECTION				0		// Direction pin value that moves the motors in the decreasing coordinate direction (from max to min)

#define Y_MIN_LIMIT_SW					(1 << 0)
#define Y_MAX_LIMIT_SW					(1 << 1)
#define X_MIN_LIMIT_SW					(1 << 2)
#define X_MAX_LIMIT_SW					(1 << 3)

#define MATCHREL_FREQ_3KHZ		899
#define MATCHREL_FREQ_2KHZ		1799
#define MATCHREL_FREQ_1KHZ		3599

#define MATCHREL_FREQ_SCALE_3KHZ(x)		(900 * x - 1)	// Scales frequency by a factor of 1/x from 3 kHz


class PlotterController {
public:
	PlotterController();

	uint8_t GetClosedLimitSwitches();
	void AssignClosedLimitSwitch(uint8_t expected_switch);

	void SetSteppingFrequency(uint16_t x_frequency, uint16_t y_frequency);
	void SetSteppingMatchRel(uint16_t x_matchrel, uint16_t y_matchrel);
	void SetXDir(bool direction);
	void SetYDir(bool direction);
	void StepX(uint16_t step_count);
	void StepY(uint16_t step_count);

	void SetSpeed(uint8_t speed_percentage);
	uint8_t speed_percentage;
	uint16_t current_x;
	uint16_t current_y;
	uint16_t x_length_mm;
	uint16_t y_length_mm;

	void SetPenPosition(uint8_t pos);
	bool IsPlotting();
	uint8_t pen_up_position;
	uint8_t pen_down_position;

	void SetLaserPower(uint8_t power);
private:
	bool is_plotting;
	uint16_t stepping_matchrel;
	void SetupStepperAxisSCT(LPC_SCT_T* pSCT, uint8_t step_pin);
	bool x_dir;
	bool y_dir;
	DigitalIoPin x_dir_pin;
	DigitalIoPin y_dir_pin;
	DigitalIoPin* limit_switches[4];		// order of the switches in the array: y_min, y_max, x_min, x_max
};

#endif /* PLOTTERCONTROLLER_H_ */
