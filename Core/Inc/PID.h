/*
 * PID.h
 *
 *  Created on: Dec 15, 2024
 *      Author: Luca Obwegs
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#define Kp_roll 0.05		// 0.1
#define Kd_roll 0.4		// 0.4
#define Ki_roll 0.000

#define Kp_pitch 0.05
#define Kd_pitch 0.4
#define Ki_pitch 0.000

#define Kp_yaw 0.03
#define Kd_yaw 0.0387
#define Ki_yaw 0.000

#define Kp_altitude 0.288
#define Kd_altitude 0.879
#define Ki_altitude 0.000

// drone parameters
// (named ARM_LENGTH/DRAG_COEFF rather than single letters `l`/`d`: those
// collide with internal identifiers in some host libm implementations,
// which breaks compiling PID.c standalone for the host-side unit tests.)
#define ARM_LENGTH 0.186
#define kt  0.0000052871
#define DRAG_COEFF (0.15*0.0000052871)
#define w_max 1976.1


struct motSpeeds {
	double w1;
	double w2;
	double w3;
	double w4;
};


struct motSpeeds callPID(float roll, float pitch, float yaw, float rollRef, float altitudeRef, float pitchRef, float yawRef, float dt);




#endif /* INC_PID_H_ */
