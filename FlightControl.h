#ifndef FLIGHTCONTROL_H_
#define FLIGHTCONTROL_H_

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <math.h>

#ifndef FLIGHTCONTROL_H_INCLUDED
#define FLIGHTCONTROL_H_INCLUDED

/* Control Parameters Default Definitions */
// PITCH
#define KPP 0.23
#define KDP 0.016
#define KIP 0.0033
//ROLL
#define KPR 0.23
#define KDR 0.016
#define KIR 0.0033
//YAW
#define KPY 0.3
#define KDY 0.01
#define KIY 0.0031
//EAST
#define KPE 1.0
#define KDE 0.01
#define KIE 0.001
//NORTH
#define KPN 1.0
#define KDN 0.01
#define KIN 0.001
//UP POSITION
#define KPZ 1.0
#define KDZ 0.0
#define KIZ 0.000
//UP RATE
#define KPDZ 2.0
#define KDDZ 0.005
#define KIDZ 0.001
/* End K params*/

#define TRUE 1 
#define FALSE 0

// FLIGHT STATES
#define CHARGE 0
#define TAKEOFF 1
#define HOVER 2
#define TRANSLATE 3
#define LAND 4

/* These Definitions Pertain to Flight Plan */
#define NumberOfModes 5
#define HOVERALTITUDE 1.5  // METERS
#define EASTTARGET 9   // METERS
#define NORTHTARGET 10  // METERS
#define LandingIndex 3 
#define MaxTakeOffSpeed 0.05 // METERS/SECOND
#define MaxLandingSpeed 0.01 // METERS/SECOND
/* End Flight Plan Definitions */ /*******************************************/

struct xyz {
	double x ; // N = x+
	double y ; // E = y+
	double z ;
} ;

struct xyzf {
    double x ;
    double y ;
    double z ;
    int f ;
} ;

/* VARIABLES */
extern double KiP, KiR, KpP, KpR, KdP, KdR, KiY, KdY, KpY;
extern double KpN, KiN, KdN, KpE, KiE, KdE;
extern double KpZ, KiZ, KdZ, KpdZ, KidZ, KddZ ;

/* FLIGHT VARIABLES */
extern int flightMode[NumberOfModes] ;
extern xyz flightCoors[NumberOfModes] ;// Number of Modes must match length of flightMode array ^
extern int flightModeIndex ;
extern xyzf initialPosition, curr_locf ;
extern xyz curr_loc, XYZ_SP, IP;
/* END FLIGHT VARIABLES */

#endif

class FlightControl {
public:
	FlightControl();
	virtual ~FlightControl();
	void initFlight();
	xyz computeXYZSetpoints(xyz fdest, xyz curr_loc, int flightMode, double yaw, xyz IP);
	xyz computeCarrotVectorXY(xyz dest, xyz curr_loc, xyz IP, double maxCarrotHoriz) ;
	xyz rotateVector(xyz Target, xyz Actual, double yaw) ;
	double computeCarrotVectorZ(xyz fdest, xyz curr_loc, double maxCarrotVert, int flightMode);
	double rotateAxes(double yawSP, double yaw) ;
	double computeGroundSpeed(xyz curr_loc, xyz last_loc, double lastTime) ;
	double computeVerticalSpeed(xyz curr_loc, xyz last_loc, double lastTime) ;
	void setInitialPosition (xyz IP) ;
private:
	xyz last_loc;
	double maxCarrotHoriz;
	double maxCarrotVertUP;
	double maxCarrotVertDOWN;
	double computeMagnitude(xyz vec, int xyOnly);
	double computeDotProduct(xyz vec1, xyz vec2, int xyOnly);
	xyz LLH_to_XYZ(xyz llh, double cs_height) ;
};

#endif /* FLIGHTCONTROL_H_ */
