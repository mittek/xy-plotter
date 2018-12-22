/*
 * DigitalIoPin.h
 *
 *  Created on: 22 Jan 2018
 *      Author: marti
 */

#ifndef DIGITALIOPIN_H_
#define DIGITALIOPIN_H_

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

#include "stdint.h"

class DigitalIoPin {
public:
	DigitalIoPin(int port, int pin, bool input = true, bool pullup = true, bool invert = false);
	virtual ~DigitalIoPin();
	bool read() const;
	void write(bool value);
private:
	int port;
	int pin;
	bool output_invert;
};

#endif /* DIGITALIOPIN_H_ */
