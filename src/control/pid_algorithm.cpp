/**
 * @file pid_algorithm.cpp
 *
 * @brief PID algorithm.
 *
 * @date 11/2023
 */

#include "control/pid_algorithm.h"

/*****************************************
 * Public Functions Bodies Definitions
 *****************************************/

PidAlgorithm::PidAlgorithm() {
    this->kp = 0;
    this->ki = 0;
    this->kd = 0;

    this->ideal_state = 0;
    this->current_state = 0;
    this->last_state = 0;

    this->integral = 0;
    this->deriv_aux = 0;

    for (uint8_t i = 0; i < DERIV_AUX_MAX + 1; i++) {
        this->last_error[i] = 0;
    }
}

float PidAlgorithm::Action() {
    float error = 0;
    float action = 0;
    float derivative = 0;

    /* PID variables update */
    error = this->ideal_state - this->current_state;
    this->integral += error;

    if (this->deriv_aux < DERIV_AUX_MAX) {
        derivative = error - this->last_error[this->deriv_aux + 1];
    } else {
        derivative = error - this->last_error[0];
    }

    this->last_error[this->deriv_aux] = error;
    this->deriv_aux = (this->deriv_aux + 1) % DERIV_AUX_MAX;

    action = this->kp * error + this->ki * this->integral + this->kd * derivative;

    return action;
}

void PidAlgorithm::SetConstants(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

void PidAlgorithm::SetIdealState(float ideal_state) {
    this->ideal_state = ideal_state;
}

void PidAlgorithm::SetCurrentState(float current_state) {
    this->current_state = current_state;
}
