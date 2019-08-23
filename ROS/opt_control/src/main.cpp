//------------------------------------------------------------------------
// File:       main.cpp
// Version:    0.1
// Maintainer: Marius Beul (mbeul@ais.uni-bonn.de)
// Package:    opt_control (https://github.com/AIS-Bonn/opt_control)
// License:    BSD
//------------------------------------------------------------------------

// Software License Agreement (BSD License)
// Copyright (c) 2018, Computer Science Institute VI, University of Bonn
// All rights reserved.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of University of Bonn, Computer Science Institute
//   VI nor the names of its contributors may be used to endorse or
//   promote products derived from this software without specific
//   prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//------------------------------------------------------------------------

#include "rt_nonfinite.h"
#include "opt_control_lib.h"
#include "opt_control_lib_terminate.h"
#include "opt_control_lib_emxAPI.h"
#include "opt_control_lib_initialize.h"
//#include <nav_msgs/Path.h>
//#include <dynamic_reconfigure/server.h>
//#include <opt_control/OptControlConfig.h>
//#include <visualization_msgs/Marker.h>
//#include "opt_control/Waypoints.h"

//#include "ros/ros.h"

#include <iostream>
#include <vector>
#include <math.h>

#include <fstream>

void exportDatas(
		double* p_init,
		double* v_init,
		double* a_init,
		std::vector<std::vector<std::vector<double>>> & wps,
		emxArray_real_T *T_waypoints,
		emxArray_real_T* t,
		emxArray_real_T* p,
		emxArray_real_T* v,
		emxArray_real_T* a,
		emxArray_real_T* j,
		std::string name
		)
{
	int rollout_iterations = p->size[1];

	std::fstream fs;
	fs.open( "data"+name+".m",  std::fstream::out );

	//declaration
	//waypoints
	fs << " wpT=zeros(2," << wps.size() + 1 << ");" << std::endl;
	fs << " wpX=zeros(2," << wps.size() + 1 << ");" << std::endl;
	fs << " wpdX=zeros(2," << wps.size() + 1 << ");" << std::endl;
	fs << " wpddX=zeros(2," << wps.size() + 1 << ");" << std::endl;
	//setpoints
	fs << " X=zeros(2," << rollout_iterations << ");" << std::endl;
	fs << " dX=zeros(2," << rollout_iterations << ");" << std::endl;
	fs << " ddX=zeros(2," << rollout_iterations << ");" << std::endl;
	fs << " dddX=zeros(2," << rollout_iterations << ");" << std::endl;
	fs << " T=zeros(1," << rollout_iterations << ");" << std::endl;

	//filling
	//waypoints
	fs << " wpT"<<"(:,1)=[0;0];"<<std::endl;
	fs << " wpX"<<"(:,1)=["<<p_init[0]<<";"<<p_init[1]<<"];"<<std::endl;
	fs << " wpdX"<<"(:,1)=["<<v_init[0]<<";"<<v_init[1]<<"];"<<std::endl;
	fs << " wpddX"<<"(:,1)=["<<a_init[0]<<";"<<a_init[1]<<"];"<<std::endl;
	std::vector<double> wpT {0.0, 0.0};
	for( int i = 0; i < wps.size(); ++i )
	{
		wpT[0]+=T_waypoints->data[0 + T_waypoints->size[0] * i];
		wpT[1]+=T_waypoints->data[1 + T_waypoints->size[0] * i];
		fs << " wpT"<<"(:,"<<i+2<<")=["<<wpT[0]<<";"<<wpT[1]<<"];"<<std::endl;
		fs << " wpX"<<"(:,"<<i+2<<")=["<<wps[i][0][0]<<";"<<wps[i][1][0]<<"];"<<std::endl;
		fs << " wpdX"<<"(:,"<<i+2<<")=["<<wps[i][0][1]<<";"<<wps[i][1][1]<<"];"<<std::endl;
		fs << " wpddX"<<"(:,"<<i+2<<")=["<<wps[i][0][2]<<";"<<wps[i][1][2]<<"];"<<std::endl;

	}
	//setpoints
	for ( int i = 0; i < rollout_iterations; ++i ) {
		fs << " X(:,"<< i + 1 << ")=[" << p->data[i * p->size[0] + 0]
			<< ";" << p->data[i * p->size[0] + 1] << "];" << std::endl;
		fs << " dX(:,"<< i + 1 << ")=[" << v->data[i * v->size[0] + 0]
			<< ";" << v->data[i * v->size[0] + 1] << "];" << std::endl;
		fs << " ddX(:,"<< i + 1 << ")=[" << a->data[i * a->size[0] + 0]
			<< ";" << a->data[i * a->size[0] + 1] << "];" << std::endl;
		fs << " dddX(:,"<< i + 1 << ")=[" << j->data[i * j->size[0] + 0]
			<< ";" << j->data[i * j->size[0] + 1] << "];" << std::endl;
		fs << " T(1,"<< i + 1 <<")=" <<  t->data[i * p->size[0] + 1] << ";" << std::endl;
	}

	fs.close();

	std::cout << "Data exported successfully !" << std::endl;
}



/* Function Definitions */
static emxArray_boolean_T *argInit_1xUnbounded_boolean_T(int _nwp)
{
	emxArray_boolean_T *result;
	static int iv182[2] = { 1, _nwp };
	int idx1;
	result = emxCreateND_boolean_T(2, iv182);
	for (idx1 = 0; idx1 < result->size[1U]; idx1++) {
		result->data[result->size[0] * idx1] = false;
	}
	return result;
}

static emxArray_real_T *argInit_Unboundedx1_real_T(int _ndof)
{
	emxArray_real_T *result;
	static int iv180[1] = { _ndof };
	int idx0;
	result = emxCreateND_real_T(1, iv180);
	for (idx0 = 0; idx0 < result->size[0U]; idx0++) {
		result->data[idx0] = 0.0;
	}
	return result;
}

static emxArray_real_T *argInit_Unboundedx3_real_T(int _ndof)
{
	emxArray_real_T *result;
	static int iv177[2] = { _ndof, 3 };
	int idx0;
	int idx1;
	result = emxCreateND_real_T(2, iv177);
	for (idx0 = 0; idx0 < result->size[0U]; idx0++) {
		for (idx1 = 0; idx1 < 3; idx1++) {
			result->data[idx0 + result->size[0] * idx1] = 0.0;
		}
	}
	return result;
}

static emxArray_int16_T *c_argInit_Unboundedx2xUnbounded(int _ndof, int _nwp)
{
	emxArray_int16_T *result;
	static int iv183[3] = { _ndof, 2, _nwp };
	int idx0;
	int idx1;
	int idx2;
	result = emxCreateND_int16_T(3, iv183);
	for (idx0 = 0; idx0 < result->size[0U]; idx0++) {
		for (idx1 = 0; idx1 < 2; idx1++) {
			for (idx2 = 0; idx2 < result->size[2U]; idx2++) {
				result->data[(idx0 + result->size[0] * idx1) + result->size[0] * result->size[1] * idx2] = 0;
			}
		}
	}
	return result;
}

static emxArray_real_T *c_argInit_Unboundedx5xUnbounded(int _ndof, int _nwp)
{
	emxArray_real_T *result;
	static int iv178[3] = { _ndof, 5, _nwp };
	int idx0;
	int idx1;
	int idx2;
	result = emxCreateND_real_T(3, iv178);
	for (idx0 = 0; idx0 < result->size[0U]; idx0++) {
		for (idx1 = 0; idx1 < 5; idx1++) {
			for (idx2 = 0; idx2 < result->size[2U]; idx2++) {
				result->data[(idx0 + result->size[0] * idx1) + result->size[0] * result->size[1] * idx2] = 0.0;
			}
		}
	}
	return result;
}

static emxArray_boolean_T *c_argInit_UnboundedxUnbounded_b(int _ndof, int _nwp)
{
	emxArray_boolean_T *result;
	static int iv181[2] = { _ndof, _nwp };
	int idx0;
	int idx1;
	result = emxCreateND_boolean_T(2, iv181);
	for (idx0 = 0; idx0 < result->size[0U]; idx0++) {
		for (idx1 = 0; idx1 < result->size[1U]; idx1++) {
			result->data[idx0 + result->size[0] * idx1] = false;
		}
	}
	return result;
}

static emxArray_real_T *c_argInit_UnboundedxUnbounded_r(int _ndof, int _nwp)
{
	emxArray_real_T *result;
	static int iv179[2] = { _ndof, _nwp };
	int idx0;
	int idx1;
	result = emxCreateND_real_T(2, iv179);
	for (idx0 = 0; idx0 < result->size[0U]; idx0++) {
		for (idx1 = 0; idx1 < result->size[1U]; idx1++) {
			result->data[idx0 + result->size[0] * idx1] = 0.0;
		}
	}
	return result;
}


//void waypoints_callback(const opt_control::Waypoints &msg)
//{
//  for (int i = 0; i < ndof; i++) {
//    P_wayp[i] = msg.P[i];
//    V_wayp[i] = msg.V[i];
//    A_wayp[i] = msg.A[i];
//  }
//}
//
//
//void dynamic_reconfigure_callback(opt_control::OptControlConfig &config, uint32_t level)
//{
//
//  V_max             = config.V_max;
//  V_min             = config.V_min;
//  A_max             = config.A_max;
//  A_min             = config.A_min;
//  J_max             = config.J_max;
//  J_min             = config.J_min;
//
//  b_sync_V          = config.b_sync_V;
//  b_sync_A          = config.b_sync_A;
//  b_sync_J          = config.b_sync_J;
//  b_sync_W          = config.b_sync_W;
//
//  b_rotate          = config.b_rotate;
//  b_best_solution   = config.b_best_solution;
//  b_hard_vel_limit  = config.b_hard_vel_limit;
//  b_catch_up        = config.b_catch_up;
//
//  ROS_INFO("Setting new parameters");
//}

//void init_int16_T(emxArray_int16_T *pEmxArray,int _ndof, int _nwp, int _numDimensions)
//{
//	static int iv183[3] = { _ndof, 2, _nwp };
//	int idx0;
//	int idx1;
//	int idx2;
//	result = emxCreateND_int16_T(3, iv183);
//	for (int idx0 = 0; idx0 < pEmxArray->size[0U]; idx0++) {
//		for (int idx1 = 0; idx1 < 2; idx1++) {
//			for (int idx2 = 0; idx2 < pEmxArray->size[2U]; idx2++) {
//				pEmxArray->data[(idx0 + pEmxArray->size[0] * idx1) + pEmxArray->size[0] * pEmxArray->size[1] * idx2] = 0;
//			}
//		}
//	}
//}


void exemple1()
{
	std::cout << "test opt_control" << std::endl;

	opt_control_lib_initialize();

	const int ndof = 2;

	std::vector<double> waypointAxis(5);
	std::vector<std::vector<double>> waypoint(ndof);
	std::vector<std::vector<std::vector<double>>> waypoints;

	// init state
	double P_init[ndof] = {-1.0, -1.0};
	double V_init[ndof] = {0.0, 0.0};
	double A_init[ndof] = {0.0, 0.0};

	// WP 1
	// Axis 1
	waypointAxis[0] = 0;	// pos
	waypointAxis[1] = NAN;	// vel
	waypointAxis[2] = NAN;	// acc
	waypointAxis[3] = 0.0;	// vel prediction
	waypointAxis[4] = 0.0;	// acc prediction
	waypoint.at(0) = waypointAxis;
	// Axis 2
	waypointAxis[0] = 1;	// pos
	waypointAxis[1] = NAN;	// vel
	waypointAxis[2] = NAN;	// acc
	waypointAxis[3] = 0;	// vel prediction
	waypointAxis[4] = 0;	// acc prediction
	waypoint.at(1) = waypointAxis;
	waypoints.push_back(waypoint);

	// WP 2
	// Axis 1
	waypointAxis[0] = 4;	// pos
	waypointAxis[1] = NAN;	// vel
	waypointAxis[2] = NAN;	// acc
	waypointAxis[3] = 0;	// vel prediction
	waypointAxis[4] = 0;	// acc prediction
	waypoint.at(0) = waypointAxis;
	// Axis 2
	waypointAxis[0] = 2;	// pos
	waypointAxis[1] = NAN;	// vel
	waypointAxis[2] = NAN;	// acc
	waypointAxis[3] = 0;	// vel prediction
	waypointAxis[4] = 0;	// acc prediction
	waypoint.at(1) = waypointAxis;
	waypoints.push_back(waypoint);

	// WP 3
	// Axis 1
	waypointAxis[0] = 5;	// pos
	waypointAxis[1] = 0;	// vel
	waypointAxis[2] = 0;	// acc
	waypointAxis[3] = 0;	// vel prediction
	waypointAxis[4] = 0;	// acc prediction
	waypoint.at(0) = waypointAxis;
	// Axis 2
	waypointAxis[0] = 5;	// pos
	waypointAxis[1] = 0;	// vel
	waypointAxis[2] = 0;	// acc
	waypointAxis[3] = 0;	// vel prediction
	waypointAxis[4] = 0;	// acc prediction
	waypoint.at(1) = waypointAxis;
	waypoints.push_back(waypoint);

	int nwp = waypoints.size();

	// constraints
	double V_max[ndof] = {1.0, 1.0};
	double V_min[ndof] = {-1.0, -1.0};
	double A_max[ndof] = {1.0, 1.0};
	double A_min[ndof] = {-1.0, -1.0};
	double J_max[ndof] = {1.0, 1.0};
	double J_min[ndof] = {-1.0, -1.0};

	// configuration gt
	double A_global = 0.0;

	bool b_sync_V = true;
	bool b_sync_A = true;
	bool b_sync_J = false;
	bool b_sync_W = false;

	bool b_comp_global = false;
	bool b_rotate = false;
	bool b_best_solution = true;
	bool b_hard_vel_limit = false;
	bool b_catch_up = false;


	//Inputs
	std::cout << "Inputs declaration" << std::endl;
	emxArray_real_T *State_start_in;
	emxArray_real_T *Waypoints_in;
	emxArray_real_T *V_max_in;
	emxArray_real_T *V_min_in;
	emxArray_real_T *A_max_in;
	emxArray_real_T *A_min_in;
	emxArray_real_T *J_max_in;
	emxArray_real_T *J_min_in;
	emxArray_real_T *A_global_in;
	emxArray_boolean_T *b_sync_V_in;
	emxArray_boolean_T *b_sync_A_in;
	emxArray_boolean_T *b_sync_J_in;
	emxArray_boolean_T *b_sync_W_in;
	emxArray_boolean_T *b_rotate_in;
	emxArray_boolean_T *b_best_solution_in;
	emxArray_boolean_T *b_hard_vel_limit_in;
	emxArray_boolean_T *b_catch_up_in;
	bool b_comp_global_in;
	emxArray_int16_T *solution_in;

	//Outputs
	std::cout << "Outputs declaration" << std::endl;
	emxArray_struct0_T *J_setp_struct;
	emxArray_int16_T *solution_out;
	emxArray_real_T *T_waypoints;
	emxArray_real_T *P_rollout;
	emxArray_real_T *V_rollout;
	emxArray_real_T *A_rollout;
	emxArray_real_T *J_rollout;
	emxArray_real_T *t_rollout;
	emxInitArray_struct0_T(&J_setp_struct, 2);
	emxInitArray_int16_T(&solution_out, 3);
	emxInitArray_real_T(&T_waypoints, 2);
	emxInitArray_real_T(&P_rollout, 2);
	emxInitArray_real_T(&V_rollout, 2);
	emxInitArray_real_T(&A_rollout, 2);
	emxInitArray_real_T(&J_rollout, 2);
	emxInitArray_real_T(&t_rollout, 2);


	std::cout << "Inputs instantiation" << std::endl;
	State_start_in = argInit_Unboundedx3_real_T(ndof);
	Waypoints_in = c_argInit_Unboundedx5xUnbounded(ndof, nwp);
	V_max_in = c_argInit_UnboundedxUnbounded_r(ndof, nwp);
	V_min_in = c_argInit_UnboundedxUnbounded_r(ndof, nwp);
	A_max_in = c_argInit_UnboundedxUnbounded_r(ndof, nwp);
	A_min_in = c_argInit_UnboundedxUnbounded_r(ndof, nwp);
	J_max_in = c_argInit_UnboundedxUnbounded_r(ndof, nwp);
	J_min_in = c_argInit_UnboundedxUnbounded_r(ndof, nwp);
	A_global_in = argInit_Unboundedx1_real_T(ndof);
	b_sync_V_in = c_argInit_UnboundedxUnbounded_b(ndof, nwp);
	b_sync_A_in = c_argInit_UnboundedxUnbounded_b(ndof, nwp);
	b_sync_J_in = c_argInit_UnboundedxUnbounded_b(ndof, nwp);
	b_sync_W_in = c_argInit_UnboundedxUnbounded_b(ndof, nwp);
	b_rotate_in = argInit_1xUnbounded_boolean_T(nwp);
	b_best_solution_in = c_argInit_UnboundedxUnbounded_b(ndof, nwp);
	b_hard_vel_limit_in = c_argInit_UnboundedxUnbounded_b(ndof, nwp);
	b_catch_up_in = c_argInit_UnboundedxUnbounded_b(ndof, nwp);
	solution_in = c_argInit_Unboundedx2xUnbounded(ndof, nwp);



	for (int idx_traj = 0; idx_traj < nwp; idx_traj++)
	{
		for (int idx_axis = 0; idx_axis < ndof; idx_axis++)
		{
			State_start_in->data[idx_axis + State_start_in->size[0] * 0] = P_init[idx_axis];
			State_start_in->data[idx_axis + State_start_in->size[0] * 1] = V_init[idx_axis];
			State_start_in->data[idx_axis + State_start_in->size[0] * 2] = A_init[idx_axis];
			Waypoints_in->data[(idx_axis + Waypoints_in->size[0] * 0)
							   + Waypoints_in->size[0] * Waypoints_in->size[1] * idx_traj]
							   = waypoints.at(idx_traj).at(idx_axis).at(0);
			Waypoints_in->data[(idx_axis + Waypoints_in->size[0] * 1)
							   + Waypoints_in->size[0] * Waypoints_in->size[1] * idx_traj]
							   = waypoints.at(idx_traj).at(idx_axis).at(1);
			Waypoints_in->data[(idx_axis + Waypoints_in->size[0] * 2)
							   + Waypoints_in->size[0] * Waypoints_in->size[1] * idx_traj]
							   = waypoints.at(idx_traj).at(idx_axis).at(2);
			Waypoints_in->data[(idx_axis + Waypoints_in->size[0] * 3)
							   + Waypoints_in->size[0] * Waypoints_in->size[1] * idx_traj]
							   = waypoints.at(idx_traj).at(idx_axis).at(3);
			Waypoints_in->data[(idx_axis + Waypoints_in->size[0] * 4)
							   + Waypoints_in->size[0] * Waypoints_in->size[1] * idx_traj]
							   = waypoints.at(idx_traj).at(idx_axis).at(4);
			V_max_in->data[idx_axis + V_max_in->size[0] * idx_traj] = V_max[idx_axis];
			V_min_in->data[idx_axis + V_min_in->size[0] * idx_traj] = V_min[idx_axis];
			A_max_in->data[idx_axis + A_max_in->size[0] * idx_traj] = A_max[idx_axis];
			A_min_in->data[idx_axis + A_min_in->size[0] * idx_traj] = A_min[idx_axis];
			J_max_in->data[idx_axis + J_max_in->size[0] * idx_traj] = J_max[idx_axis];
			J_min_in->data[idx_axis + J_min_in->size[0] * idx_traj] = J_min[idx_axis];
			A_global_in->data[idx_axis]  = A_global;
			b_comp_global_in             = b_comp_global;
			b_sync_V_in->data[idx_axis + b_sync_V_in->size[0] * idx_traj] = b_sync_V;
			b_sync_A_in->data[idx_axis + b_sync_A_in->size[0] * idx_traj] = b_sync_A;
			b_sync_J_in->data[idx_axis + b_sync_J_in->size[0] * idx_traj] = b_sync_J;
			b_sync_W_in->data[idx_axis + b_sync_W_in->size[0] * idx_traj] = b_sync_W;
			b_rotate_in->data[b_rotate_in->size[0] * idx_axis]                            = b_rotate;
			b_best_solution_in->data[idx_axis + b_best_solution_in->size[0] * idx_traj]   = b_best_solution;
			b_hard_vel_limit_in->data[idx_axis + b_hard_vel_limit_in->size[0] * idx_traj] = b_hard_vel_limit;
			b_catch_up_in->data[idx_axis + b_catch_up_in->size[0] * idx_traj]             = b_catch_up;
		}
	}


	double ts_rollout = 0.1;

	double t_timing_start = clock();
	opt_control_lib(State_start_in, Waypoints_in, V_max_in, V_min_in, A_max_in, A_min_in, J_max_in, J_min_in,
			A_global_in, b_comp_global_in, b_sync_V_in, b_sync_A_in, b_sync_J_in, b_sync_W_in, b_rotate_in,
			b_best_solution_in, b_hard_vel_limit_in, b_catch_up_in, solution_in, ts_rollout, J_setp_struct,
			solution_out, T_waypoints, P_rollout, V_rollout, A_rollout, J_rollout, t_rollout);

	double t_timing_stop = clock();

	printf("Elapsed time is %.2fms\n",(t_timing_stop - t_timing_start)/CLOCKS_PER_SEC*1000);



	int rollout_iterations = P_rollout->size[1];
	printf("rollout_iterations = %i\n", rollout_iterations);

	std::cout << "T_waypoints size = " << T_waypoints->size[0] << " " << T_waypoints->size[1] << std::endl;
	for (int idx_traj = 0; idx_traj < nwp; idx_traj++)
	{
		for (int idx_axis = 0; idx_axis < ndof; idx_axis++)
		{
			std::cout << "wp : " << idx_traj << ", ndof : " << idx_axis
					<< ", t = " << T_waypoints->data[idx_axis + T_waypoints->size[0] * idx_traj] << std::endl;
		}
	}

	std::cout << "P_rollout size = " << P_rollout->size[0] << " " << P_rollout->size[1] << std::endl;
	for (int idx_traj = 0; idx_traj < nwp; idx_traj++)
	{
		for (int idx_axis = 0; idx_axis < ndof; idx_axis++)
		{
			std::cout << "wp : " << idx_traj << ", ndof : " << idx_axis
					<< ", t = " << T_waypoints->data[idx_axis + T_waypoints->size[0] * idx_traj] << std::endl;
		}
	}

//    for ( int i = 0; i < rollout_iterations; ++i ) {
//		std::cout << "axis n, t : " << t_rollout->data[i * P_rollout->size[0] + 1] << std::endl;
//		std::cout << "axis 1, p : " << P_rollout->data[i * P_rollout->size[0] + 0] << std::endl;
//		std::cout << "axis 2, p : " << P_rollout->data[i * P_rollout->size[0] + 1] << std::endl;
//		std::cout << "axis 1, v : " << V_rollout->data[i * V_rollout->size[0] + 0] << std::endl;
//		std::cout << "axis 2, v : " << V_rollout->data[i * V_rollout->size[0] + 1] << std::endl;
//		std::cout << "axis 1, a : " << A_rollout->data[i * A_rollout->size[0] + 0] << std::endl;
//		std::cout << "axis 2, a : " << A_rollout->data[i * A_rollout->size[0] + 1] << std::endl;
//    }

    exportDatas( P_init, V_init, A_init, waypoints, T_waypoints,
    		t_rollout, P_rollout, V_rollout, A_rollout, J_rollout, "1" );


	emxDestroyArray_real_T(J_rollout);
	emxDestroyArray_real_T(A_rollout);
	emxDestroyArray_real_T(V_rollout);
	emxDestroyArray_real_T(P_rollout);
	emxDestroyArray_real_T(t_rollout);
	emxDestroyArray_real_T(T_waypoints);
	emxDestroyArray_int16_T(solution_out);
	emxDestroyArray_struct0_T(J_setp_struct);
	emxDestroyArray_int16_T(solution_in);
	emxDestroyArray_boolean_T(b_catch_up_in);
	emxDestroyArray_boolean_T(b_hard_vel_limit_in);
	emxDestroyArray_boolean_T(b_best_solution_in);
	emxDestroyArray_boolean_T(b_rotate_in);
	emxDestroyArray_boolean_T(b_sync_W_in);
	emxDestroyArray_boolean_T(b_sync_J_in);
	emxDestroyArray_boolean_T(b_sync_A_in);
	emxDestroyArray_boolean_T(b_sync_V_in);
	emxDestroyArray_real_T(A_global_in);
	emxDestroyArray_real_T(J_min_in);
	emxDestroyArray_real_T(J_max_in);
	emxDestroyArray_real_T(A_min_in);
	emxDestroyArray_real_T(A_max_in);
	emxDestroyArray_real_T(V_min_in);
	emxDestroyArray_real_T(V_max_in);
	emxDestroyArray_real_T(Waypoints_in);
	emxDestroyArray_real_T(State_start_in);

	opt_control_lib_terminate();

}


int main(int argc, char **argv)
{
	exemple1();
}

