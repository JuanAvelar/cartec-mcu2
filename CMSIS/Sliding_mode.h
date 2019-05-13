/******************************************************************************
 * @file     Sliding_mode.h
 * @brief    Sliding mode motor control
 * @author	 Juan Avelar
 * @author	 Miguel Omar Ortiz
 * @date     13. May 2019
 ******************************************************************************/

/*
 * Sliding_mode.h
 *
 *  Created on: 13/05/2019
 *      Author: Juan Avelar
 */

#ifndef SLIDING_MODE_H_
#define SLIDING_MODE_H_
#include "arm_math.h"

  /**
   * @brief Instance structure for the floating-point PID Control.
   */
  typedef struct
  {
    float32_t sigma;          /**< sliding surface */
    float32_t u;          /**< control output */
    float32_t W;          /**< integral control section */
    float32_t C;           /**< The sliding surface gain. */
    float32_t C1;          /**< The proportional gain. */
    float32_t B;           /**< The integral gain. */
    float32_t previous_error; /**< saving previous error */
    float32_t time_step;
  } arm_STC_instance_f32;

  void init_STC_f32(arm_STC_instance_f32 * S, float32_t ts);
  float32_t arm_STC_f32(arm_STC_instance_f32 * S, float32_t error);



#endif /* SLIDING_MODE_H_ */
