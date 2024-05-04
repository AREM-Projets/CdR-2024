/*
 * Mouvement3Roues.h
 *
 *  Created on: May 1, 2024
 *      Author: Kezia
 */

#ifndef INC_EMBASE3ROUES_HPP_
#define INC_EMBASE3ROUES_HPP_

#include <stdint.h>

#include "BlocMoteurs.hpp"
#include "config.hpp"

extern volatile bool motors_busy;

enum TaskType_t
{
	NONE,
	WAIT,
	MOVE_RELATIVE
};

struct Task_t
{
	TaskType_t type;

	// Type : movement (relative or not...)
	double x;
	double y;
	double theta;

	// Type : wait
	uint32_t delay_ms;
};

class Embase3Roues : public BlocMoteurs
{
public:
	using BlocMoteurs::BlocMoteurs;
	void init();

	int32_t appendRelativeMove(double x, double y, double theta);
	int32_t appendWait(uint32_t delay_ms);

	TaskType_t executeInstruction();

	int32_t getCurrentIndex();
	TaskType_t getCurrentType();

private:
	Task_t _task_buffer[MAX_TASK_COUNT];
	int32_t _last_index = 0;

	int32_t _current_index = 0;

	int32_t appendInstruction(Task_t task);

	// Movement related, used by moveRelative()
	void setStep(double x, double y, double theta);
	void translate(double x, double y);
	void rotate(double theta);

	// Functions linked to tasks
	void moveRelative(double x, double y, double theta);
	void wait(uint32_t delay_ms);
};

void initTask(Task_t &task);

#endif /* INC_EMBASE3ROUES_HPP_ */
