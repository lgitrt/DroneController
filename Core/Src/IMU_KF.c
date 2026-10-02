/*
 * IMU_KF.c
 *
 *  Created on: Nov 25, 2024
 *      Author: Luca Obwegs
 */

#include "IMU_KF.h"

// tuning parameters
double Qphi = 0.001;
double Qtheta = 0.001;
double Qpsi = 0.001;
double Rphi = 0.03;
double Rtheta = 0.03;
double Rpsi = 0.03;


// initialization
double phiS = 0;
double PnPhi = 0;
double PpPhi = 0;
double KPhi = 0;
double xkkPhi = 0;
double xhatPhi = 0;

double thetaS = 0;
double PnTheta = 0;
double PpTheta = 0;
double KTheta = 0;
double xkkTheta = 0;
double xhatTheta = 0;

double psiS = 0;
double PnPsi = 0;
double PpPsi = 0;
double KPsi = 0;
double xkkPsi = 0;
double xhatPsi = 0;

double alpha = 0.02;
double pitchComp = 0;
double rollComp = 0;

struct est estOut;
struct est estComp;

struct est callKF(double ax, double ay, double az, double gx, double gy, double gz, double ts) {
	// acc in g, gyro in dps
	phiS = atan2(ay, (az));
	thetaS = atan2(ax, (az));

	PnPhi = PpPhi + Qphi;
	KPhi = PnPhi/(PnPhi + Rphi);
	xkkPhi = xhatPhi - ts*gy*pi/180;
	xhatPhi = xkkPhi+KPhi*(phiS-xkkPhi);
	PpPhi = (1-KPhi)*PnPhi;

	PnTheta = PpTheta + Qtheta;
	KTheta = PnTheta/(PnTheta + Rtheta);
	xkkTheta = xhatTheta + ts*gx*pi/180;
	xhatTheta = xkkTheta+KTheta*(thetaS-xkkTheta);
	PpTheta = (1-KTheta)*PnTheta;

	estOut.phi = xhatPhi;
	estOut.theta = xhatTheta;

	return estOut;

}

/*
struct est callKF(double ax, double ay, double az, double gx, double gy, double gz, double mx, double my, double mz, double ts) {
    // Accelerometer-based roll and pitch
    double phiS = atan2(ay, az);
    double thetaS = atan2(ax, az);

    // Magnetometer-based yaw
    double psiS = atan2(my, mx);

    // Roll (phi) Kalman Filter
    PnPhi = PpPhi + Qphi;
    KPhi = PnPhi / (PnPhi + Rphi);
    xkkPhi = xhatPhi - ts * gy * M_PI / 180;
    xhatPhi = xkkPhi + KPhi * (phiS - xkkPhi);
    PpPhi = (1 - KPhi) * PnPhi;

    // Pitch (theta) Kalman Filter
    PnTheta = PpTheta + Qtheta;
    KTheta = PnTheta / (PnTheta + Rtheta);
    xkkTheta = xhatTheta + ts * gx * M_PI / 180;
    xhatTheta = xkkTheta + KTheta * (thetaS - xkkTheta);
    PpTheta = (1 - KTheta) * PnTheta;

    // Yaw (psi) Kalman Filter
    PnPsi = PpPsi + Qpsi;
    KPsi = PnPsi / (PnPsi + Rpsi);
    xkkPsi = xhatPsi + ts * gz * M_PI / 180;
    xhatPsi = xkkPsi + KPsi * (psiS - xkkPsi);
    PpPsi = (1 - KPsi) * PnPsi;

    // Update the output structure
    estOut.phi = xhatPhi;
    estOut.theta = xhatTheta;
    estOut.psi = xhatPsi;

    return estOut;
}*/

struct est callCompFilter(double ax, double ay, double az, double gx, double gy, double gz, double ts) {
	phiS = atan2(ay, (az));
	thetaS = atan2(ax, (az));
	pitchComp = (1-alpha)*(pitchComp+gx*ts*pi/180)+alpha*thetaS;
	rollComp = (1-alpha)*(rollComp+gx*ts*pi/180)+alpha*phiS;

	estComp.phi = rollComp;
	estComp.theta = pitchComp;

	return estComp;
}


