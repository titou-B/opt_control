/*
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * opt_control.h
 *
 * Code generation for function 'opt_control'
 *
 */

#ifndef OPT_CONTROL_H
#define OPT_CONTROL_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "opt_control_lib_types.h"

/* Function Declarations */
extern void opt_control(emxArray_real_T *State_start, const emxArray_real_T
  *Waypoints, const emxArray_real_T *V_max_in, const emxArray_real_T *V_min_in,
  emxArray_real_T *A_max_in, emxArray_real_T *A_min_in, const emxArray_real_T
  *J_max_in, const emxArray_real_T *J_min_in, const emxArray_real_T *A_global,
  bool b_comp_global, const emxArray_boolean_T *b_sync_V_in, const
  emxArray_boolean_T *b_sync_A_in, const emxArray_boolean_T *b_sync_J_in, const
  emxArray_boolean_T *b_sync_W_in, const emxArray_boolean_T *b_coll_prev_in,
  const emxArray_boolean_T *b_rotate_in, const emxArray_boolean_T
  *b_best_solution_in, const emxArray_boolean_T *b_hard_vel_limit_in, const
  emxArray_boolean_T *b_catch_up, const emxArray_real_T *Obstacles, const
  emxArray_real_T *MAV_d, const emxArray_real_T *MAV_margin, const
  emxArray_real_T *axis_penalty, const emxArray_int16_T *solution_in,
  emxArray_struct0_T *J_setp_struct, emxArray_int16_T *solution_out,
  emxArray_real_T *T_waypoints);

#endif

/* End of code generation (opt_control.h) */
