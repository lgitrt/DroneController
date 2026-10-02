/*
 * PID.c
 *
 *  Created on: Dec 15, 2024
 *      Author: Luca Obwegs
 */

#include "PID.h"
#include <math.h>

struct motSpeeds omega;
float e_roll_old = 0; float e_pitch_old = 0; float e_yaw_old = 0; float e_altitude_old = 0;
float rollCtr = 0; float pitchCtr = 0; float yawCtr = 0; float altitudeCtr = 0;
float e_roll = 0; float e_pitch = 0; float e_yaw = 0; float e_altitude = 0;
float Tc = 0.001;
float e_roll_der_filt = 0; float e_roll_der_filt_old = 0;
float e_pitch_der_filt = 0; float e_pitch_der_filt_old = 0;


struct motSpeeds callPID(float roll, float pitch, float yaw, float rollRef, float altitudeRef, float pitchRef, float yawRef, float dt) {

	// roll PID
	e_roll = rollRef - roll;
	e_roll_der_filt = (e_roll - e_roll_old) /dt;
	e_roll_der_filt = Tc*(e_roll_der_filt) + (1-Tc)*e_roll_der_filt_old;
	rollCtr = Kp_roll*e_roll + Kd_roll*e_roll_der_filt;
	e_roll_old = e_roll;
	e_roll_der_filt_old = e_roll_der_filt;

	// pitch PID
	e_pitch = pitchRef - pitch;
	e_pitch_der_filt = (e_pitch - e_pitch_old) /dt;
	e_pitch_der_filt = Tc*(e_pitch_der_filt) + (1-Tc)*e_pitch_der_filt_old;
	pitchCtr = Kp_pitch*e_pitch + Kd_pitch*e_pitch_der_filt;
	e_pitch_old = e_pitch;
	e_pitch_der_filt_old = e_pitch_der_filt;

	// yaw PID
	//e_yaw = yawRef - yaw;
	//yawCtr = Kp_yaw*e_yaw; // + Kd_yaw*(e_yaw - e_yaw_old)/dt;
	//e_yaw_old = e_yaw;
	yawCtr = yawRef*Kp_yaw;

	//printf("%f\t%f\t%f\t\n", e_roll, e_pitch, e_yaw);

	// altitude PID
	//e_altitude = altitudeRef - altitude;
	//altitudeCtr = Kp_altitude*e_altitude + Kd_altitude*(e_altitude - e_altitude_old)/dt;
	//e_altitude_old = e_altitude;
	altitudeCtr = altitudeRef;

	// motor mixing algorithm
	omega.w1 = (yawCtr/(4*DRAG_COEFF) + altitudeCtr/(4*kt) + rollCtr/(2*kt*ARM_LENGTH));
	omega.w2 = (altitudeCtr/(4*kt) - yawCtr/(4*DRAG_COEFF) + pitchCtr/(2*kt*ARM_LENGTH));
	omega.w3 = (yawCtr/(4*DRAG_COEFF) + altitudeCtr/(4*kt) - rollCtr/(2*kt*ARM_LENGTH));
	omega.w4 = (altitudeCtr/(4*kt) - yawCtr/(4*DRAG_COEFF) - pitchCtr/(2*kt*ARM_LENGTH));

	// only use positive values
	if (omega.w1 < 0) {omega.w1 = 0;}
	if (omega.w2 < 0) {omega.w2 = 0;}
	if (omega.w3 < 0) {omega.w3 = 0;}
	if (omega.w4 < 0) {omega.w4 = 0;}

	// square root
	omega.w1 = sqrt(omega.w1);
	omega.w2 = sqrt(omega.w2);
	omega.w3 = sqrt(omega.w3);
	omega.w4 = sqrt(omega.w4);

	// speed limiting
	if (omega.w1 > w_max) {omega.w1 = w_max;}
	if (omega.w2 > w_max) {omega.w2 = w_max;}
	if (omega.w3 > w_max) {omega.w3 = w_max;}
	if (omega.w4 > w_max) {omega.w4 = w_max;}

	return omega;
}
