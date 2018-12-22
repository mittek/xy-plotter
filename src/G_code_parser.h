/*
 * G_code_parser.h
 *
 *  Created on: 28 Sep 2018
 *      Author: marti
 */

#ifndef G_CODE_PARSER_H_
#define G_CODE_PARSER_H_

#include "cstring"
#include "cmath"
#include "cstdlib"
#include "PlotterController.h"

typedef enum {
	GOTO_HOME,				// G28
	LINEAR_MOVE,			// G1
	PEN_MOVE,				// M1
	SAVE_PEN_POS,			// M2
	LASER_POWER,			// M4
	SAVE_SETTINGS,			// M5
	RETURN_SETTINGS,		// M10
	LIMIT_SWITCH_STATUS,	// M11
	CALIBRATE				// M28
} G_CODE_COMMAND;

typedef enum {
	NO_ERROR,
	UNKNOWN_COMMAND
} G_CODE_ERROR;

typedef struct {
	G_CODE_COMMAND command;
	int command_data[3];		// if linear move, X - Y - A
	G_CODE_ERROR error_flag;
} G_CODE_BLOCK;


void ParseBlock (char* line, G_CODE_BLOCK* block);


#endif /* G_CODE_PARSER_H_ */
