/*
 * G_code_parser.cpp
 *
 *  Created on: 28 Sep 2018
 *      Author: marti
 */


#include "G_code_parser.h"

// G-code parser.cpp : Defines the entry point for the console application.
//

//#define UNIT_TEST

#ifdef UNIT_TEST

#include "iostream"
#include "sstream"
#include "iomanip"
#include "fstream"
#include "assert.h"
#include "string"

#else


#endif


void ParseGCode (char* line, G_CODE_BLOCK* block);
void ParseMCode (char* line, G_CODE_BLOCK* block);

#ifdef UNIT_TEST

std::string GetReconstructedBlockString (G_CODE_BLOCK* block);

int main()
{
	// 1) Reads contents of a file containing g-code commands, line by line
	// 2) Parses the line (g-code block) and saves the results to a struct
	// 3) Reconstructs the line, using the struct
	// 4) Compares the original line and reconstructed line
	//	  If they do not match, displays an error and the problematic line can be seen in the console

	std::ifstream g_code_commands("gcode01.txt");
	std::string file_line;
	std::cout << "Starting UNIT TEST" << std::endl;

	G_CODE_BLOCK test_block;

	char c_line[50];
	while (std::getline(g_code_commands, file_line)) {
		strcpy(c_line, file_line.c_str());
		ParseBlock(c_line, &test_block);
		std::string block_string = GetReconstructedBlockString(&test_block);
		std::cout << block_string << std::endl;
		assert(file_line == block_string);
	}

	std::cout << std::endl << "UNIT TEST finished, no errors found" << std::endl;

	system("pause");
    return 0;
}

#endif

void ParseBlock (char* line, G_CODE_BLOCK* block) {
	// Read the first letter of the line and send the rest to G- or M-code parser
	block->error_flag = NO_ERROR;
	switch (line[0]) {
		case 'G':
			ParseGCode(line + 1, block);
			break;
		case 'M':
			ParseMCode(line + 1, block);
			break;
		default:
			block->error_flag = UNKNOWN_COMMAND;
	}
}

void ParseGCode (char* line, G_CODE_BLOCK* block) {
	char* tok;
	tok = strtok(line, " ");
	int code = atoi(tok);

	switch (code) {
		case 1:		// Linear movement
			block->command = LINEAR_MOVE;
			tok = strtok(NULL, " ");
			while (tok != NULL) {
				switch (tok[0]) {
					// Convert x and y coordinates to integers
					case 'X':
						block->command_data[0] = (int)nearbyint((atof(tok + 1) * STEPS_PER_MM));
						break;
					case 'Y':
						block->command_data[1] = (int)nearbyint((atof(tok + 1) * STEPS_PER_MM));
						break;
					case 'A':
						block->command_data[2] = atoi(tok + 1);
						break;
				}
				tok = strtok(NULL, " ");
			}
			break;
		case 28:	// Go to to home position
			block->command = GOTO_HOME;
			break;
		default:
			block->error_flag = UNKNOWN_COMMAND;
			break;
	}
}

void ParseMCode (char* line, G_CODE_BLOCK* block) {
	char* tok;
	tok = strtok(line, " ");
	int code = atoi(tok);

	switch (code) {
		case 1:		// Move pen up/down
			block->command = PEN_MOVE;
			tok = strtok(NULL, " ");
			block->command_data[0] = atoi(tok);
			break;
		case 2:
			block->command = SAVE_PEN_POS;
			tok = strtok(NULL, " ");
			block->command_data[0] = atoi(tok + 1);
			tok = strtok(NULL, " ");
			block->command_data[1] = atoi(tok + 1);
			break;
		case 4:		// Laser power
			block->command = LASER_POWER;
			tok = strtok(NULL, " ");
			block->command_data[0] = atoi(tok);
			break;
		case 5:		// Save XY-plotter settings from mDraw
			block->command = SAVE_SETTINGS;
			tok = strtok(NULL, " ");
			while (tok != NULL) {
				switch (tok[0]) {
				case 'A':
					// Not used
					break;
				case 'B':
					// Not used
					break;
				case 'H':
					block->command_data[0] = atoi(tok + 1);
					break;
				case 'W':
					block->command_data[1] = atoi(tok + 1);
					break;
				case 'S':
					block->command_data[2] = atoi(tok + 1);
					break;
				}
				tok = strtok(NULL, " ");
			}
			break;
		case 10:	// Send XY-plotter settings to mDraw
			block->command = RETURN_SETTINGS;
			break;
		case 11:
			block->command = LIMIT_SWITCH_STATUS;
			break;
		case 28:
			block->command = CALIBRATE;
			break;
		default:
			block->error_flag = UNKNOWN_COMMAND;
			break;
	}
}

#ifdef UNIT_TEST

std::string GetReconstructedBlockString (G_CODE_BLOCK* block) {
	std::stringstream block_stream;
	switch (block->command) {
		case LINEAR_MOVE:
			block_stream << std::fixed << std::setprecision(2) << "G1 X" << block->command_data[0] / 100.0 << " Y" << block->command_data[1] / 100.0 << " A" << block->command_data[2];
			break;
		case GOTO_HOME:
			block_stream << "G28";
			break;
		case PEN_MOVE:
			block_stream << "M1 " << std::to_string(block->command_data[0]);
			break;
		case LASER_POWER:
			block_stream << "M4 " << std::to_string(block->command_data[0]);
			break;
		case RETURN_SETTINGS:
			block_stream << "M10";
			break;
	}
	return block_stream.str();
}

#endif

