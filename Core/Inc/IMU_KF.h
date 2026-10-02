/*
 * IMU_KF.h
 *
 *  Created on: Nov 25, 2024
 *      Author: OBWELUC
 */

#ifndef INC_IMU_KF_H_
#define INC_IMU_KF_H_

#define pi 3.1416

#include <stdio.h>
#include <stdlib.h>
#include "math.h"

struct est {
	double phi;
	double theta;
	double psi;
};

struct est callKF(double ax, double ay, double az, double gx, double gy, double gz, double ts);
struct est callCompFilter(double ax, double ay, double az, double gx, double gy, double gz, double ts);


#endif /* INC_IMU_KF_H_ */
