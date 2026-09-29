/**
 * @file pid_algorithm.cpp
 *
 * @brief PID algorithm.
 *
 * @date 11/2023
 */

#ifndef __PID_ALGORITHM_H__
#define __PID_ALGORITHM_H__

#include <stdint.h>

/*****************************************
 * Public Constants
 *****************************************/

/**
 * @brief last_error array number of positions.
 */
#define DERIV_AUX_MAX 20

/*****************************************
 * Class Methods Body Definitions
 *****************************************/

class PidAlgorithm {
    public:
        /**
         * @brief Initialize pid service instance.
         */
        PidAlgorithm();

        /**
         * @brief Calculates the action of the pid algorithm.
         *
         * @param pid Pointer to pid package of variables
         *            used for calculating the action.
         *
         * @return pid action.
         */
        float Action();

        /**
         * @brief Sets PID algorithm constants.
         *
         * @param pid Pointer to pid package of variables
         *            used for calculating the action.
         * @param kp Base PID proportional constant.
         * @param ki Base PID integral constant.
         * @param kd Base PID derivative constant.
         */
        void SetConstants(float kp, float ki, float kd);

        /**
         * @brief Sets PID algorithm ideal state.
         *
         * @param pid Pointer to pid package of variables
         *            used for calculating the action.
         * @param ideal_state Base ideal state to be used.
         */
        void SetIdealState(float ideal_state);

        /**
         * @brief Sets PID algorithm current state.
         *
         * @param pid Pointer to pid package of variables
         *            used for calculating the action.
         * @param current_state Current state of the robot position.
         */
        void SetCurrentState(float current_state);

    private:
        float kp;  /**< Proportional constant. */
        float ki;  /**< Integral constant. */
        float kd;  /**< Derivative constant. */

        float ideal_state;   /**< Ideal state of the control variable. */
        float current_state; /**< Current state of the control variable. */
        float last_state;    /**< Last state of the control variable. */

        float integral;
        uint8_t deriv_aux;  /**< Auxiliary variable to improve the derivative. Queue index */
        float last_error[DERIV_AUX_MAX];
};

#endif // __PID_ALGORITHM_H__
