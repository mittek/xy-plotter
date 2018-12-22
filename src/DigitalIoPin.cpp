/*
 * DigitalIoPin.cpp
 *
 *  Created on: 22 Jan 2018
 *      Author: marti
 */

#include "DigitalIoPin.h"

DigitalIoPin::DigitalIoPin(int port, int pin, bool input, bool pullup, bool invert): port(port), pin(pin), output_invert(false) {
	if (input) {
		if (pullup) {
			if (invert) {
				Chip_IOCON_PinMuxSet(LPC_IOCON, port, pin, IOCON_MODE_PULLUP | IOCON_DIGMODE_EN | IOCON_INV_EN);
			} else {
				Chip_IOCON_PinMuxSet(LPC_IOCON, port, pin, IOCON_MODE_PULLUP | IOCON_DIGMODE_EN);
			}
		} else {
			if (invert) {
				Chip_IOCON_PinMuxSet(LPC_IOCON, port, pin, IOCON_MODE_PULLDOWN | IOCON_DIGMODE_EN | IOCON_INV_EN);
			} else {
				Chip_IOCON_PinMuxSet(LPC_IOCON, port, pin, IOCON_MODE_PULLDOWN | IOCON_DIGMODE_EN);
			}
		}
		Chip_GPIO_SetPinDIRInput(LPC_GPIO, port, pin);
	} else {
		Chip_IOCON_PinMuxSet(LPC_IOCON, port, pin, IOCON_MODE_INACT | IOCON_DIGMODE_EN);
		Chip_GPIO_SetPinDIROutput(LPC_GPIO, port, pin);
		output_invert = invert;
	}
}

DigitalIoPin::~DigitalIoPin() {
}

bool DigitalIoPin::read() const {
	return (output_invert ^ Chip_GPIO_GetPinState(LPC_GPIO, port, pin));
}

void DigitalIoPin::write(bool value) {
	Chip_GPIO_SetPinState(LPC_GPIO, port, pin, output_invert ^ value);
}
