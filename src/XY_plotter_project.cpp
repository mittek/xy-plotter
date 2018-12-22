/*
===============================================================================
 Name        : main.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
 */

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

#include <cr_section_macros.h>

#include "FreeRTOS.h"
#include "task.h"
#include "ITM_write.h"

#include <mutex>
#include "Fmutex.h"
#include "user_vcom.h"
#include "cstdlib"
#include "string.h"

#include "G_code_parser.h"
#include "PlotterController.h"
//#include "DigitalIoPin.h"

#define MAX_RECEIVED_CHARS 60


PlotterController* plotter;

QueueHandle_t command_queue;
SemaphoreHandle_t x_steps_done;
SemaphoreHandle_t y_steps_done;


void CalibratePlottingArea();
void MoveMotors(int delta_x, int delta_y);

extern "C" {
void SCT1_IRQHandler (void) {
	portBASE_TYPE xHigherPriorityWoken = pdFALSE;
	Chip_SCT_ClearEventFlag(LPC_SCT1, SCT_EVT_1);
	xSemaphoreGiveFromISR(x_steps_done, &xHigherPriorityWoken);
	portEND_SWITCHING_ISR(xHigherPriorityWoken);
}

void SCT2_IRQHandler (void) {
	portBASE_TYPE xHigherPriorityWoken = pdFALSE;
	Chip_SCT_ClearEventFlag(LPC_SCT2, SCT_EVT_1);
	xSemaphoreGiveFromISR(y_steps_done, &xHigherPriorityWoken);
	portEND_SWITCHING_ISR(xHigherPriorityWoken);
}
void GINT0_IRQHandler (void) {
	Chip_GPIOGP_ClearIntStatus(LPC_GPIOGROUP, 0);
	bool hit = true;
	// Limit switches seem to trigger without actually closing them
	// This loop eliminates the false readings by polling the switches 10 times in a row
	for (int i = 0; i < 10; i++) {
		if (plotter->GetClosedLimitSwitches() == 0) {
			hit = false;
		}
	}
	if (hit) {	// Limit switch was actually closed, so halt the timers (this state is not recoverable)
		LPC_SCT1->CTRL_U |= SCT_CTRL_HALT_L | SCT_CTRL_HALT_H;
		LPC_SCT2->CTRL_U |= SCT_CTRL_HALT_L | SCT_CTRL_HALT_H;
		printf("Group interrupt triggered and timers stopped \r\n");
	} else {	// False reading
		printf("Group interrupt triggered but timers not stopped \r\n");
	}
}
}



static void prvSetupHardware(void)
{
	SystemCoreClockUpdate();
	Board_Init();
}

static void vControllerTask(void* pvParameters) {

	DigitalIoPin dir_pin_y(0, 28, false);
	DigitalIoPin dir_pin_x(1, 0, false);

	G_CODE_BLOCK received_command;
	bool calibrated = false;
	int delta_y;
	int delta_x;

	while (1) {

		xQueueReceive(command_queue, &received_command, portMAX_DELAY);

		switch (received_command.command) {
		case LINEAR_MOVE:
			delta_x = received_command.command_data[0] - plotter->current_x;
			delta_y = received_command.command_data[1] - plotter->current_y;
			if (delta_x != 0 || delta_y != 0) {
				plotter->current_x = received_command.command_data[0];
				plotter->current_y = received_command.command_data[1];
				MoveMotors(delta_x, delta_y);
			}
			break;
		case PEN_MOVE:
			plotter->SetPenPosition(received_command.command_data[0]);
			vTaskDelay(200);		// mDraw adds 200ms delay after every pen move command but can't be used as the queue doesn't save delays
			break;
		case LASER_POWER:
			plotter->SetLaserPower(received_command.command_data[0]);
			break;
		case GOTO_HOME:
			delta_x = 0 - plotter->current_x;
			delta_y = 0 - plotter->current_y;
			if (delta_x != 0 || delta_y != 0) {
				plotter->current_x = 0;
				plotter->current_y = 0;
				MoveMotors(delta_x, delta_y);
			}
			break;
		case SAVE_PEN_POS:
			plotter->pen_up_position = received_command.command_data[0];
			plotter->pen_down_position = received_command.command_data[1];
			USB_send((uint8_t*) "Pen positions saved\n", 20);
			break;
		case LIMIT_SWITCH_STATUS:
			{
				char buf[60];
				uint8_t status = ~(plotter->GetClosedLimitSwitches());		// mDraw requests inverted values
				int len = sprintf(buf, "M11 %d %d %d %d\n",
						(bool) (status & X_MIN_LIMIT_SW),
						(bool) (status & X_MAX_LIMIT_SW),
						(bool) (status & Y_MIN_LIMIT_SW),
						(bool) (status & Y_MAX_LIMIT_SW));
				USB_send((uint8_t*) buf, len);
			}
			break;
		case CALIBRATE:
			CalibratePlottingArea();
			calibrated = true;
			// Intentional fallthrough
		case RETURN_SETTINGS:
			if (calibrated) {
				char buf[60];
				int len = sprintf(buf, "M10 XY %d %d 0.00 0.00 A0 B0 H0 S%d U%d D%d\n", plotter->x_length_mm, plotter->y_length_mm, plotter-> speed_percentage, plotter->pen_up_position, plotter->pen_down_position);
				USB_send((uint8_t*) buf, len);
			} else {					// Initial message sent to mDraw as the COM port is opened
				USB_send((uint8_t*) "Calibrate the plotter using M28 command\n", 40);
				USB_send((uint8_t*) "NOTE: recommended speed when using the laser is 0%\n", 51);
			}
			break;
		case SAVE_SETTINGS:
			plotter->y_length_mm = received_command.command_data[0];
			plotter->x_length_mm = received_command.command_data[1];
			plotter->SetSpeed(received_command.command_data[2]);
			break;
		}

	}	// Infinite while loop

}	// Task


static void vmDrawReceiveTask(void *pvParameters) {

	char received_buf[64];
	char received_str[MAX_RECEIVED_CHARS];
	uint32_t received_len = 0;
	uint32_t total_received_len = 0;

	G_CODE_BLOCK received_g_code;

	while (1) {

		received_len = USB_receive((uint8_t*) (received_buf), 64);

		for (uint32_t i = 0; i < received_len; i++) {
			received_str[total_received_len + i] = received_buf[i];
			if (received_buf[i] == '\n' || (total_received_len + i) == (MAX_RECEIVED_CHARS - 1)) {
				received_str[total_received_len + i] = '\0';
				ParseBlock(received_str, &received_g_code);
				xQueueSendToBack(command_queue, &received_g_code, portMAX_DELAY);
				USB_send((uint8_t*)"OK\n", 3);
				received_len = 0;
				total_received_len = 0;
			}
		}

		total_received_len += received_len;

	}

}



int main(void) {

	prvSetupHardware();
	ITM_init();

	plotter = new PlotterController();

	command_queue = xQueueCreate(20, sizeof(G_CODE_BLOCK));
	x_steps_done = xSemaphoreCreateBinary();
	y_steps_done = xSemaphoreCreateBinary();

	xTaskCreate(cdc_task, "CDC", 200, NULL, (tskIDLE_PRIORITY + 3UL), (TaskHandle_t *) NULL);
	xTaskCreate(vmDrawReceiveTask, "mDraw Task", 600, NULL, (tskIDLE_PRIORITY + 1UL), (TaskHandle_t *) NULL);
	xTaskCreate(vControllerTask, "Cntrl Task", 600, NULL, (tskIDLE_PRIORITY + 2UL), (TaskHandle_t *) NULL);

	vTaskStartScheduler();

	return 1;
}



void MoveMotors(int delta_x, int delta_y) {

	if (delta_x > 0) {
		plotter->SetXDir(POSITIVE_XY_DIRECTION);
	} else {
		plotter->SetXDir(NEGATIVE_XY_DIRECTION);
		delta_x *= -1;
	}

	if (delta_y > 0) {
		plotter->SetYDir(POSITIVE_XY_DIRECTION);
	} else {
		plotter->SetYDir(NEGATIVE_XY_DIRECTION);
		delta_y *= -1;
	}

	if (!plotter->IsPlotting()) {	// Check if the plotter is actually drawing on the paper or just moving around without drawing

		// If the plotter is not drawing, speed up the motors
		plotter->SetSteppingMatchRel(MATCHREL_FREQ_3KHZ, MATCHREL_FREQ_3KHZ);
		if ( (delta_x == 0 || delta_y == 0) || (delta_x == delta_y) ) {
			plotter->StepX(delta_x);
			plotter->StepY(delta_y);
			xSemaphoreTake(x_steps_done, portMAX_DELAY);
			xSemaphoreTake(y_steps_done, portMAX_DELAY);
		} else {
			// Frequency scaling instead of Bresenham, bit shaky but faster
			//		One of the motors is slowed down so they both finish at the same time even if one has to step more,
			//		this results in a diagonal line
			if (delta_x > delta_y) {
				plotter->SetSteppingMatchRel(MATCHREL_FREQ_3KHZ, MATCHREL_FREQ_SCALE_3KHZ(delta_x / delta_y));
				plotter->StepX(delta_x);
				plotter->StepY(delta_y);
				xSemaphoreTake(x_steps_done, portMAX_DELAY);
				xSemaphoreTake(y_steps_done, portMAX_DELAY);
			} else {
				plotter->SetSteppingMatchRel(MATCHREL_FREQ_SCALE_3KHZ(delta_y / delta_x), MATCHREL_FREQ_3KHZ);
				plotter->StepX(delta_x);
				plotter->StepY(delta_y);
				xSemaphoreTake(x_steps_done, portMAX_DELAY);
				xSemaphoreTake(y_steps_done, portMAX_DELAY);
			}
		}
		plotter->SetSpeed(plotter->speed_percentage);

	} else {

		// Algorithm not needed if the step counts are equal (due to both motors running at the same time)
		// 		or if only one motor has to step
		if ( (delta_x == 0 || delta_y == 0) || (delta_x == delta_y) ) {

			plotter->StepX(delta_x);
			plotter->StepY(delta_y);
			xSemaphoreTake(x_steps_done, portMAX_DELAY);
			xSemaphoreTake(y_steps_done, portMAX_DELAY);

		} else { 	// Use Bresenham's algorithm

			if (delta_x > delta_y) {

				int p0 = 2 * delta_y - delta_x;
				int x_steps = 0;
				for (int i = 0; i < delta_x; i++, x_steps++) {
					if (p0 < 0) {
						p0 = p0 + 2 * delta_y;
					} else {
						plotter->StepX(x_steps);
						x_steps = 0;
						p0 = p0 + 2 * delta_y - 2 * delta_x;
						xSemaphoreTake(x_steps_done, portMAX_DELAY);
						plotter->StepY(1);
						xSemaphoreTake(y_steps_done, portMAX_DELAY);
					}
				}
				plotter->StepX(x_steps);
				xSemaphoreTake(x_steps_done, portMAX_DELAY);

			} else {

				int p0 = 2 * delta_x - delta_y;
				int y_steps = 0;
				for (int i = 0; i < delta_y; i++, y_steps++) {
					if (p0 < 0) {
						p0 = p0 + 2 * delta_x;
					} else {
						plotter->StepY(y_steps);
						y_steps = 0;
						p0 = p0 + 2 * delta_x - 2 * delta_y;
						xSemaphoreTake(y_steps_done, portMAX_DELAY);
						plotter->StepX(1);
						xSemaphoreTake(x_steps_done, portMAX_DELAY);
					}
				}
				plotter->StepY(y_steps);
				xSemaphoreTake(y_steps_done, portMAX_DELAY);

			}

		}	// Bresenham's algorithm
	}	// Pen / laser check

}	// Function


void CalibratePlottingArea() {

	NVIC_DisableIRQ(GINT0_IRQn);

	plotter->current_x = 0;
	plotter->current_y = 0;
	plotter->x_length_mm = 0;
	plotter->y_length_mm = 0;


	/*
	 * If one or more of the limit switches is closed before the calibration, the limit switch position checking won't work.
	 * Therefore, notify the user to manually release all limit switches before continuing.
	 */

	if (plotter->GetClosedLimitSwitches() > 0) {
		USB_send((uint8_t*) "All limit switches have to be open before calibration\n", 54);
		while (plotter->GetClosedLimitSwitches() > 0);
		vTaskDelay(1000);
	}


	/*
	 * Run the plotter in one direction until a limit switch is hit. Then read the switches to find the closed one and save its position.
	 * This is done 4 times to find all the limits.
	 */


	// Find x max (L4), assign it, and then release it
	plotter->SetXDir(POSITIVE_XY_DIRECTION);
	while (plotter->GetClosedLimitSwitches() == 0) {
		plotter->StepX(1);
		xSemaphoreTake(x_steps_done, portMAX_DELAY);
	}
	plotter->AssignClosedLimitSwitch(4);

	plotter->SetXDir(NEGATIVE_XY_DIRECTION);
	while (plotter->GetClosedLimitSwitches() > 0) {
		plotter->StepX(10);
		xSemaphoreTake(x_steps_done, portMAX_DELAY);
	}
	plotter->StepX(100);
	xSemaphoreTake(x_steps_done, portMAX_DELAY);


	// Find x min (L4), assign it, and then release it
	while (plotter->GetClosedLimitSwitches() == 0) {
		plotter->StepX(1);
		plotter->x_length_mm++;
		xSemaphoreTake(x_steps_done, portMAX_DELAY);
	}
	plotter->AssignClosedLimitSwitch(3);

	plotter->SetXDir(POSITIVE_XY_DIRECTION);
	while (plotter->GetClosedLimitSwitches() > 0) {
		plotter->StepX(10);
		plotter->x_length_mm -= 10;
		xSemaphoreTake(x_steps_done, portMAX_DELAY);
	}
	plotter->StepX(100);
	plotter->x_length_mm -= 100;
	plotter->x_length_mm /= STEPS_PER_MM;

	xSemaphoreTake(x_steps_done, portMAX_DELAY);

	// Find y max (L2), assign it, and then release it
	plotter->SetYDir(POSITIVE_XY_DIRECTION);
	while (plotter->GetClosedLimitSwitches() == 0) {
		plotter->StepY(1);
		xSemaphoreTake(y_steps_done, portMAX_DELAY);
	}
	plotter->AssignClosedLimitSwitch(2);

	plotter->SetYDir(NEGATIVE_XY_DIRECTION);
	while (plotter->GetClosedLimitSwitches() > 0) {
		plotter->StepY(10);
		xSemaphoreTake(y_steps_done, portMAX_DELAY);
	}
	plotter->StepY(100);
	xSemaphoreTake(y_steps_done, portMAX_DELAY);

	// y min (L1) should be correctly assigned; just count the steps
	while (plotter->GetClosedLimitSwitches() == 0) {
		plotter->StepY(1);
		plotter->y_length_mm++;
		xSemaphoreTake(y_steps_done, portMAX_DELAY);
	}

	plotter->SetYDir(POSITIVE_XY_DIRECTION);
	while (plotter->GetClosedLimitSwitches() > 0) {
		plotter->StepY(10);
		plotter->y_length_mm -= 10;
		xSemaphoreTake(y_steps_done, portMAX_DELAY);
	}
	plotter->StepY(100);
	plotter->y_length_mm -= 100;
	plotter->y_length_mm /= STEPS_PER_MM;

	xSemaphoreTake(y_steps_done, portMAX_DELAY);

	printf("Limit switches assigned, x length: %d y length %d \r\n", plotter->x_length_mm, plotter->y_length_mm);

	// Limit switches hit during calibration cause an interrupt request. Clear it so the interrupt handler doesn't get falsely called
	Chip_GPIOGP_ClearIntStatus(LPC_GPIOGROUP, 0);
	NVIC_ClearPendingIRQ(GINT0_IRQn);
	NVIC_EnableIRQ(GINT0_IRQn);

}
