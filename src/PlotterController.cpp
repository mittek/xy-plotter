/*
 * PlotterController.cpp
 *
 *  Created on: Oct 14, 2018
 *      Author: Martin
 */

#include "PlotterController.h"


// Public functions

PlotterController::PlotterController() :
				speed_percentage(99),
				pen_up_position(100),
				pen_down_position(30),
				is_plotting(false),
				stepping_matchrel(MATCHREL_FREQ_2KHZ),
				x_dir_pin(X_DIR_PORT, X_DIR_PIN, false),
				y_dir_pin(Y_DIR_PORT, Y_DIR_PIN, false),
				limit_switches {
					new DigitalIoPin (0, 0, true, true, true),
					new DigitalIoPin (1, 3, true, true, true),
					new DigitalIoPin (0, 29, true, true, true),
					new DigitalIoPin (0, 9, true, true, true)
				}
{

	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);			// Enable clock to the switch matrix to assign SCT outputs


	/*
	 * Configure the x and y axis SCTs
	 *
	 * SCT peripherals are clocked by system clock (72 MHz in this case) or by SCT PLL (PLL input 12 MHz in this case -> PLL output 24 - 72+ MHz)
	 * Therefore, lowest possible clock for the SCT peripheral without under-clocking the CPU is 24 MHz with SCT PLL
	 * High clock speed causes the counter to overflow too quickly, limiting the max number of steps that can be sent with a single register write
	 * To overcome this issue, SCT0 is used to generate a low frequency clock (20 kHz) for SCT1 and SCT2
	 */

	// SCT0 to generate clock for SCT1 and SCT2
	// 72 MHz SCT clock, toggle output every 1800 counts (1 wavelength = 3600 counts) to generate 20 kHz clock

	Chip_SCT_Init(LPC_SCT0);
	LPC_SCT0->CONFIG =
			SCT_CONFIG_16BIT_COUNTER |
			SCT_CONFIG_AUTOLIMIT_L |
			SCT_CONFIG_AUTOLIMIT_H;

	// Clock for SCT1
	LPC_SCT0->MATCH[0].L = 1800 - 1;						// toggle output every 1800 counts to get 20 kHz output toggle
	LPC_SCT0->MATCHREL[0].L = 1800 - 1;

	LPC_SCT0->EVENT[0].STATE = 1 << 0;
	LPC_SCT0->EVENT[0].CTRL = 0 | (1 << 12);				// Triggered only by a counter match in register MATCH[0].L

	LPC_SCT0->OUT[4].SET = 1 << 0;							// EVENT 0 sets and clears the output at the same time, use conflict resolution to toggle the output
	LPC_SCT0->OUT[4].CLR = 1 << 0;
	LPC_SCT0->RES |= SCT_RES_TOGGLE_OUTPUT << 8;

	// Clock for SCT2
	LPC_SCT0->MATCH[0].H = 1800 - 1;
	LPC_SCT0->MATCHREL[0].H = 1800 - 1;

	LPC_SCT0->EVENT[1].STATE = 1 << 0;
	LPC_SCT0->EVENT[1].CTRL = 0  | (1 << 4)| (1 << 12);		// Triggered only by a counter match in register MATCH[0].H

	LPC_SCT0->OUT[5].SET = 1 << 1;
	LPC_SCT0->OUT[5].CLR = 1 << 1;
	LPC_SCT0->RES |= SCT_RES_TOGGLE_OUTPUT << 10;

	// Connect SCT0 outputs to SCT1 and SCT2 inputs using input multiplexer
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_MUX);
	Chip_INMUX_SelectSCT1Src(0, SCT1_INMUX_SCT0_OUT4);
	Chip_INMUX_SelectSCT2Src(0, SCT2_INMUX_SCT0_OUT5);
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_MUX);

	SetupStepperAxisSCT(LPC_SCT1, X_STEP_PIN);
	SetupStepperAxisSCT(LPC_SCT2, Y_STEP_PIN);

	LPC_SCT0->CTRL_U &= ~(SCT_CTRL_HALT_L | SCT_CTRL_HALT_H);	// Start the clock


	/*
	 * Configure the pen servo SCT
	 */

	Chip_SCT_Init(LPC_SCT3);

	LPC_SCT3->CONFIG = SCT_CONFIG_16BIT_COUNTER | SCT_CONFIG_AUTOLIMIT_L;
	LPC_SCT3->CTRL_U |= SCT_CTRL_PRE_L(72-1) | SCT_CTRL_CLRCTR_L;

	LPC_SCT3->MATCH[0].L = 20000 - 1;						// 50 Hz PWM frequency
	LPC_SCT3->MATCHREL[0].L = 20000 - 1;
	LPC_SCT3->EVENT[0].STATE = 1 << 0;
	LPC_SCT3->EVENT[0].CTRL = 0 | (1 << 12);				// Event triggered only by a counter match in MATCH[0].L
	LPC_SCT3->OUT[0].SET = 1 << 0;							// Event 0 sets the output high

	LPC_SCT3->MATCH[1].L = 1637;							// Default duty cycle ~8%
	LPC_SCT3->MATCHREL[1].L = 1637;
	LPC_SCT3->EVENT[1].STATE = 1 << 0;
	LPC_SCT3->EVENT[1].CTRL = 1 | (1 << 12);				// Event triggered only by a counter match in MATCH[1].L
	LPC_SCT3->OUT[0].CLR = 1 << 1;							// Event 1 sets the output low

	Chip_SWM_MovablePinAssign(SWM_SCT3_OUT0_O, PEN_PIN);

	LPC_SCT3->CTRL_L &= ~SCT_CTRL_HALT_L;


	/*
	 * Configure the laser SCT
	 * Sharing SCT3 with the pen servo
	 *
	 * Centre aligned PWM
	 * Writing 0 to MATCHREL[1].H sets the output to low: counter is counting up so EVENT[2] only clears the output; event is triggered only once so no conflicts occur
	 * Writing 255 to MATCHREL[1].H sets the output to high: counter is counting down so EVENT[2] only sets the output
	 */

	LPC_SCT3->CONFIG |= SCT_CONFIG_AUTOLIMIT_H;
	LPC_SCT3->CTRL_U |= SCT_CTRL_PRE_H(10 - 1) | SCT_CTRL_CLRCTR_H | SCT_CTRL_BIDIR_H(1);	// 7.2 MHz clock, bidirectional counter

	LPC_SCT3->MATCH[0].H = 255;								// ~14 kHz PWM frequency
	LPC_SCT3->MATCHREL[0].H = 255;
	LPC_SCT3->MATCH[1].H = 0;								// Set the output to low by default
	LPC_SCT3->MATCHREL[1].H = 0;

	LPC_SCT3->EVENT[2].STATE = 1 << 0;
	LPC_SCT3->EVENT[2].CTRL = 1 | (1 << 4) | (1 << 12);		// Event is triggered only by a counter match in MATCH[1].H

	LPC_SCT3->OUTPUTDIRCTRL = 0x2 << 2;						// Set and clear are reversed when counting down
	LPC_SCT3->OUT[1].CLR = 1 << 2;							// Event 2 sets output 1 low when counting up and high when counting down

	Chip_SWM_MovablePinAssign(SWM_SCT3_OUT1_O, LASER_PIN);

	LPC_SCT3->CTRL_U &= ~SCT_CTRL_HALT_H;


	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);		// Disable clock to the switch matrix


	/*
	 * Initialise the direction pins
	 */

	x_dir_pin.write(0);
	x_dir = false;
	y_dir_pin.write(0);
	y_dir = false;


	/*
	 * Configure the group interrupt used by the limit switches
	 * Interrupt is requested if one or more of the limit switches are closed
	 */

	Chip_GPIOGP_Init(LPC_GPIOGROUP);
	LPC_GPIOGROUP->CTRL = 0 << 1 | 0 << 2;						// OR the inputs; edge sensitive
	LPC_GPIOGROUP->PORT_ENA[0] = 1 << 0 | 1 << 9 | 1 << 29;		// enable port 0, pins 0, 9, 29
	LPC_GPIOGROUP->PORT_ENA[1] = 1 << 3;						// enable port 1 pin 3
	NVIC_SetPriority(GINT0_IRQn, LIMIT_SWITCH_INTERRUPT_PRIORITY);
	NVIC_EnableIRQ(GINT0_IRQn);

}



/*
 * Returns:
 * 		If a switch is closed, a corresponding bit is set in the return value
 * 		Bit 0:	y min
 * 		Bit 1:	y max
 * 		Bit 2:	x min
 * 		Bit 3:	x max
 * 		Returned value is 0 if all limit switches are open and greater than 1 if one or more of the limit switches are closed
 */
uint8_t PlotterController::GetClosedLimitSwitches() {

	uint8_t result = 0;
	for (uint8_t i = 0; i <= 3; i++) {
		result |= limit_switches[i]->read() << i;
	}
	return result;

}

/*
 * Assigns currently closed limit switch to the one specified by the expected switch parameter
 * Checks if the currently closed limit switch is the same as the specified expected_switch; assumes exactly one limit switch is closed
 * If any other than the expected switch is closed, swap the pointers of the closed and expected switch
 * Expected switch:
 * 		1	y min
 * 		2	y max
 * 		3	x min
 * 		4	x max
 */
void PlotterController::AssignClosedLimitSwitch(uint8_t expected_switch) {

	expected_switch -= 1;
	for (uint8_t i = 0; i <= 3; i++) {						// Check all the available limit switches (except the expected switch) if they are closed.
		if (i != expected_switch && limit_switches[i]->read() == true) {
			DigitalIoPin* buffer;
			buffer = limit_switches[i];
			limit_switches[i] = limit_switches[expected_switch];
			limit_switches[expected_switch] = buffer;
			break;
		}
	}

}


void PlotterController::SetSteppingFrequency(uint16_t x_frequency, uint16_t y_frequency) {

	// Sets the frequency of the input clock to the stepping SCTs. Stepping SCTs divide the input clock by 10 to get the actual stepping frequency.
	// FORMULA: matchrel = ( sct clock / ( _frequency * 10 ) ) / 2 - 1 = (72 000 000 / ( frequency * 10 ) ) / 2 - 1 = 3 600 000 / frequency - 1
	//LPC_SCT0->MATCHREL[0].L = 3600000 / x_frequency - 1;
	//LPC_SCT0->MATCHREL[0].H = 3600000 / y_frequency - 1;
	SetSteppingMatchRel(3600000 / x_frequency - 1, 3600000 / y_frequency - 1);

}

/*
 * Set the drawing speed of the plotter in percents from 1 to 2 kHz
 * 		Speed:
 * 			0:	1kHz frequency
 * 			99:	2kHz frequency		(max mDraw speed)
 */
void PlotterController::SetSpeed(uint8_t speed) {

	speed_percentage = speed;
	uint16_t matchrel = 1799 + (99 - speed) * 18;			// matchrel: 1khz 3599, 2khz 1799 (higher matchrel = lower speed)
	SetSteppingMatchRel(matchrel, matchrel);				// min speed is actually 1.005 kHz

}

/*
 * Allows high resolution individual frequency setting of the motors
 *		Choosing x_matchrel / y_matchrel value:
 *			matchrel = 3 600 000 / frequency - 1
 */
void PlotterController::SetSteppingMatchRel(uint16_t x_matchrel, uint16_t y_matchrel) {

	LPC_SCT0->MATCHREL[0].L = x_matchrel;
	LPC_SCT0->MATCHREL[0].H = y_matchrel;

}


/*
 * Direction:
 * 		TRUE:  counter-clockwise direction, moves the motor in the positive (increasing coordinate value) direction
 * 		FALSE: clockwise direction, moves the motor in the negative (decreasing coordinate value) direction
 */
void PlotterController::SetXDir(bool direction) {

	if (x_dir != direction) {
		x_dir_pin.write(direction);
		x_dir = direction;
	}

}

/*
 * Direction:
 * 		TRUE:  counter-clockwise direction, moves the motor in the positive (increasing coordinate value) direction
 * 		FALSE: clockwise direction, moves the motor in the negative (decreasing coordinate value) direction
 */
void PlotterController::SetYDir(bool direction) {

	if (y_dir != direction) {
		y_dir_pin.write(direction);
		y_dir = direction;
	}

}

/*
 * Step X axis motor by step_count amount
 * MAX step count: 65 536
 * MIN step count: 0
 */
void PlotterController::StepX(uint16_t step_count) {

	if (step_count != 0) {											// 0 step count will break the counting
		LPC_SCT1->MATCH[0].H = step_count - 1;						// Optimised for max step count with single function call: assumes match.l = 4 and high counter runs 10 times slower than low counter
		LPC_SCT1->CTRL_U &= ~(SCT_CTRL_HALT_L | SCT_CTRL_HALT_H);
	} else {
		NVIC_SetPendingIRQ(SCT1_IRQn);
	}

	// MATCH[0].H FORMULA to stop the low counter:
	//		Both timers running at the same speed
	// 		match.h = 2 * (match.l + 1) * step_count - 1
	// 		If match.l = 4
	// 		match.h = 10 * step_count - 1
	//		If high counter runs 10 times slower than low counter
	//		match.h = step_count - 1

}

/*
 * Step Y axis motor by step_count amount
 * MAX step count: 65 536
 * MIN step count: 0
 */
void PlotterController::StepY(uint16_t step_count) {

	if (step_count != 0) {
		LPC_SCT2->MATCH[0].H = step_count - 1;
		LPC_SCT2->CTRL_U &= ~(SCT_CTRL_HALT_L | SCT_CTRL_HALT_H);
	} else {
		NVIC_SetPendingIRQ(SCT2_IRQn);
	}

}

/*
 * Set the duty cycle of the pen servo
 * Position:
 * 		0 to 255 where 0 is 5% duty cycle and 255 is 10% duty cycle
 */
void PlotterController::SetPenPosition(uint8_t position) {

	if (position == pen_up_position)
		is_plotting = false;
	else
		is_plotting = true;
	LPC_SCT3->MATCHREL[1].L = (uint16_t) ((1000 * position)/255 + 999);		// FORMULA: (1 + pos/255.0) * 1000 - 1

}

/*
 * Returns TRUE if the pen is up / laser is off and FALSE if the pen is down / laser is on
 */
bool PlotterController::IsPlotting() {

	return is_plotting;

}

/*
 * Set the PWM duty cycle of the laser
 * Power:
 * 		0 to 255 where 0 is 0% duty cycle and 255 is 100% duty cycle
 * 		0 turns the laser completely off
 */
void PlotterController::SetLaserPower(uint8_t power) {

	if (power == 0)
		is_plotting = false;
	else
		is_plotting = true;
	LPC_SCT3->MATCHREL[1].H = power;

}


// Private functions

// Function used to configure the X and Y SC timers
void PlotterController::SetupStepperAxisSCT(LPC_SCT_T* pSCT, uint8_t step_pin) {

	// Low counter generates pulses
	// High counter stops the low counter (halts both)
	// 		FORMULA: output frequency = input frequency / (2 * MATCHREL[0].L)


	// Base configuration
	Chip_SCT_Init(pSCT);
	pSCT->CONFIG =
			SCT_CONFIG_16BIT_COUNTER |				// 2 timers
			SCT_CONFIG_CLKMODE_INCLK |				// Input 0 generates the clock signal for the timer which comes from SCT0
			SCT_CONFIG_NORELOADL_U |				// Match reload registers not used for low counter
			SCT_CONFIG_NORELOADH |					// Match reload registers not used for high counter
			SCT_CONFIG_AUTOLIMIT_L |
			SCT_CONFIG_AUTOLIMIT_H;

	pSCT->CTRL_U |=  SCT_CTRL_PRE_H(10 - 1) | SCT_CTRL_CLRCTR_H;	// high counter runs 10 times slower than low counter


	// Step generation

	/*

	                       +----------+
	                       |          |
	                       |          |
	                       |          |
	            +----------+          +----------+
	            ^          ^          ^
	    count   0    MATCHREL[0].L    2 * MATCHREL[0].L

	 */

	pSCT->MATCH[0].L = 5 - 1;
	pSCT->EVENT[0].STATE = 1 << 0;
	pSCT->EVENT[0].CTRL = 0 | (1 << 12);			// Triggered only by a counter match in MATCH[0].L
	pSCT->OUT[0].SET = 1 << 0;						// Event 0 sets and ...
	pSCT->OUT[0].CLR = 1 << 0;						//					... clears the output
	pSCT->RES |= SCT_RES_TOGGLE_OUTPUT;				// Simultaneous set and clear toggles the output


	// Step counter and stop condition

	pSCT->EVENT[1].STATE = 1;
	pSCT->EVENT[1].CTRL = 0 | (1 << 4) | (1 << 12);		// triggered only by a counter match in MATCH[0].H
	pSCT->HALT = 1 << 1 | 1 << 17;						// event 1 halts low and high counters
	pSCT->EVEN = 1 << 1;								// event 1 requests an interrupt


	// Connect SCT output 0 to the step pin and enable interrupts in NVIC

	if (pSCT == LPC_SCT1) {
		Chip_SWM_MovablePinAssign(SWM_SCT1_OUT0_O, step_pin);
		NVIC_SetPriority(SCT1_IRQn, SCT_INTERRUPT_PRIORITY);
		NVIC_EnableIRQ(SCT1_IRQn);
	} else if (pSCT == LPC_SCT2) {
		Chip_SWM_MovablePinAssign(SWM_SCT2_OUT0_O, step_pin);
		NVIC_SetPriority(SCT2_IRQn, SCT_INTERRUPT_PRIORITY);
		NVIC_EnableIRQ(SCT2_IRQn);
	}

}


