/*
 * Embase3Roues.cpp
 *
 *  Created on: May 1, 2024
 *      Author: Kezia
 */

#include "Embase3Roues.hpp"

/*******************************
 * Public functions
 ******************************/

/**
 * @brief Initialize all tasks at NONE, and reset task counts...
 * 
 */
void Embase3Roues::init() {
	for (int32_t i = 0; i < MAX_TASK_COUNT; ++i) {
		initTask(_task_buffer[i]);
	}

	_last_index = 0;
	_current_index = 0;
}

/**
 * @brief Append a movement task at the end of the queue
 * 
 * @param x 
 * @param y 
 * @param theta 
 * @return int32_t : the index of the added task, or -1 in case of failure (queue full)
 */
int32_t Embase3Roues::appendRelativeMove(double x, double y, double theta) {
	Task_t task;
	initTask(task);

	task.type = MOVE_RELATIVE;

	task.x = x;
	task.y = y;
	task.theta = theta;

	int32_t res = appendInstruction(task);
	return res;
}

/**
 * @brief Append a wait task at the end of the queue
 * 
 * @param delay_ms 
 * @return int32_t : the index of the added task, or -1 in case of failure (queue full)
 */
int32_t Embase3Roues::appendWait(uint32_t delay_ms) {
	Task_t task;
	initTask(task);

	task.type = WAIT;

	task.delay_ms = delay_ms;

	int32_t res = appendInstruction(task);
	return res;
}

/**
 * @brief Insert a movement task as the next task
 *
 * @param x
 * @param y
 * @param theta
 * @return int32_t : the index of the added task, or -1 in case of failure (queue full)
 */
int32_t Embase3Roues::insertRelativeMove(double x, double y, double theta) {
	Task_t task;
	initTask(task);

	task.type = MOVE_RELATIVE;

	task.x = x;
	task.y = y;
	task.theta = theta;

	int32_t res = insertInstruction(task);
	return res;
}

/**
 * @brief Insert a wait task as the next task
 *
 * @param delay_ms
 * @return int32_t : the index of the added task, or -1 in case of failure (queue full)
 */
int32_t Embase3Roues::insertWait(uint32_t delay_ms) {
	Task_t task;
	initTask(task);

	task.type = WAIT;

	task.delay_ms = delay_ms;

	int32_t res = insertInstruction(task);
	return res;
}


/**
 * @brief Execute an instruction from the queue
 * 
 * @return int32_t : the index of the executed instruction
 */
TaskType_t Embase3Roues::executeInstruction() {
	Task_t current_task = _task_buffer[_current_index];
	TaskType_t type = current_task.type;

	// "delete" current task
	initTask(_task_buffer[_current_index]);

	// Only go forward in the buffer if the task was not NONE.
	if (type != NONE)
	{
		_current_index++;
	}

	switch (type) {
	case NONE:
		// Do nothing...
		break;

	case MOVE_RELATIVE:
		moveRelative(current_task.x, current_task.y, current_task.theta);
		break;

	case WAIT:
		wait(current_task.delay_ms);
		break;

	default:
		break;
	}

	return type;
}

/**
 * @brief Get the index of the next instruction
 * 
 * @return int32_t 
 */
int32_t Embase3Roues::getCurrentIndex() {
	return _current_index;
}

/**
 * @brief Get the type of the current instruction (the one that will be excuted next)
 *
 */
TaskType_t Embase3Roues::getCurrentType()
{
	return _task_buffer[_current_index].type;
}

/*******************************
 * Private functions
 ******************************/

void copyTask(Task_t &dest, Task_t &src)
{
	dest.type = src.type;
	dest.x = src.x;
	dest.y = src.y;
	dest.theta = src.theta;
	dest.delay_ms = src.delay_ms;
}

void initTask(Task_t &task)
{
	task.type = TaskType_t::NONE;
	task.x = 0;
	task.y = 0;
	task.theta = 0;
	task.delay_ms = 0;
}

int32_t Embase3Roues::appendInstruction(Task_t task) {
	if (_task_buffer[_last_index].type != NONE) {
		// The task queue is full - error
	 	return -1;
	}
	if(task.type == NONE)
	{
		// Nothing happened
		return _last_index;
	}

	copyTask(_task_buffer[_last_index], task);

	_last_index = (_last_index + 1) % MAX_TASK_COUNT;

	return _last_index;
}

int32_t Embase3Roues::insertInstruction(Task_t task)
{

	if(task.type == NONE)
	{
		// Nothing happened
		return _current_index;
	}

	int32_t i = (_current_index + 1) % MAX_TASK_COUNT;
	while(i != _current_index)
	{
		copyTask(_task_buffer[i], _task_buffer[(i - 1) % MAX_TASK_COUNT]);

	}

	copyTask(_task_buffer[_current_index], task);
	_last_index = (_last_index + 1) % MAX_TASK_COUNT;


	return _current_index;
}

void Embase3Roues::setStep(double x, double y, double theta) {
	//Conversion en m
	x *= 4; //Conversion pour que Set-position(1,0,0) donne une translation de 1m suivant X
	y *= 4; //Conversion pour que Set-position(0,1,0) donne une translation de 1m suivant Y
	theta *= 0.070777; //Conversion pour que Set_Position(0,0,360) donne une rotation de 360° autour de Z

	//Angles en radians
	double theta_a = 1 / Rayon_Roue * (coeff_1x * x * 0.5 - coeff_1y * y * coeff_Y - diametre_embase * theta * coeff_1z);
	double theta_c = 1 / Rayon_Roue * (coeff_3x * x * 0.5 + y * coeff_Y * coeff_3y - diametre_embase * theta * coeff_3z);
	double theta_b = 1 / Rayon_Roue * (coeff_2x * 1 * x - diametre_embase * theta * coeff_2z);

	double step_a = rad_to_step(theta_a);
	double step_b = rad_to_step(theta_b);
	double step_c = rad_to_step(theta_c);

	StepperMotor::direction_t dir_a = FWD;
	StepperMotor::direction_t dir_b = FWD;
	StepperMotor::direction_t dir_c = FWD;

	//Gestion des directions par défaut: avancer sinon, si nb de step negatif, inversion du nb de steps et direction arriere
	if (step_a < 0) {
		step_a = 0 - step_a;
		dir_a = BWD;
	}
	if (step_b < 0) {
		step_b = 0 - step_b;
		dir_b = BWD;
	}
	if (step_c < 0) {
		step_c = 0 - step_c;
		dir_c = BWD;
	}

	if (step_a || step_b || step_c)
	{
		while(!movement_allowed);
		motors_busy = true;
		commande_step_indiv(step_a, dir_a, step_b, dir_b, step_c, dir_c, 0, FWD);
		while(motors_busy); // Wait for motors to be off.
	}

}

void Embase3Roues::translate(double x, double y) {

	// passage en coordonnées polaires.
	double distance = 0;
	double theta = 0;

	if((y == 0)&&(x != 0))
	{
		if(x < 0)
		{
			theta = -90;
			distance = -x;

		}
		else
		{
			theta = 90;
			distance = x;

		}

	}
	else if((y != 0) && (x == 0))
	{
		if(y < 0)
		{
			theta = 180;
			distance = -y;

		}
		else
		{
			theta = 0;
			distance = y;

		}
	}
	else
	{
		distance = sqrt(pow(x * Coeff_erreur_x, 2) + pow(y * Coeff_erreur_y, 2));//Distance à parcourir en m
		theta = atan2(x, y) * 180.0 / M_PI;//Angle vers lequel s'orienté en °
	}

	// Execution des mouvements
	rotate(theta); // Rotation initiale

	while(distance > BASE_MOVEMENT_DIST_M)
	{
		setStep(0, BASE_MOVEMENT_DIST_M, 0); // Découpage des mouvements
		distance -= BASE_MOVEMENT_DIST_M;
	}

	while(!movement_allowed);
	setStep(0, distance, 0); // Mouvement final

	rotate(-theta);
}

void Embase3Roues::rotate(double theta) {
	setStep(0, 0, theta * Coeff_erreur_z);
}

void Embase3Roues::moveRelative(double x, double y, double theta) {
	motors_on();
	translate(x, y);
	rotate(theta);
	motors_stop_hard();
}

void Embase3Roues::wait(uint32_t delay_ms) {
	motors_stop_hard();
	HAL_Delay(delay_ms);
}

