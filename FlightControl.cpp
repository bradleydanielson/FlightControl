/*
  FlightControl.cpp
  Written by Brad Danielson, Suzanne Reisberg, and Rudy Hulse
  Controls all aspects of flight
*/
#include "FlightControl.h"

/* Definitions*/
#define maxCarrotHorizDefault 0.5 // Max Magnitude of Horizontal Heading Vector
#define maxCarrotVertDefaultUP 0.05
#define maxCarrotVertDefaultDOWN 0.03 
#define TRUE 1 
#define FALSE 0
//#define PI 3.14159265358979323846  /* pi */
#define deg2rad 0.0174533



/*flightMode... 
    0 = CHARGE
    1 = TAKEOFF
    2 = HOVER
    3 = TRANSLATE
    4 = LAND
*/



/* VARIABLES */
double KiP = KIP, KiR = KIR, KpP = KPP, KpR = KPR, KdP = KDP, KdR = KDR, KiY = KIY, KdY = KDY, KpY = KPY;
double KpN = KPN, KiN = KIN, KdN = KDN, KpE = KPE, KiE = KIE, KdE = KDE;
double KpZ = KPZ, KiZ = KIZ, KdZ = KDZ, KpdZ = KPDZ, KidZ = KIDZ, KddZ = KDDZ ;

/* FLIGHT VARIABLES */
int flightMode[NumberOfModes] =  {CHARGE, TAKEOFF, HOVER,  LAND,  CHARGE} ;
xyz flightCoors[NumberOfModes] ;// Number of Modes must match length of flightMode array ^
int flightModeIndex = 0 ;
xyzf initialPosition, curr_locf ;
xyz curr_loc, XYZ_SP, IP;
	double maxCarrotHoriz = maxCarrotHorizDefault ;
	double maxCarrotVertUP  = maxCarrotVertDefaultUP ;
	double maxCarrotVertDOWN  = maxCarrotVertDefaultDOWN ;
/* END FLIGHT VARIABLES */


/* Constructor */
FlightControl::FlightControl() {
    /* Initial Values Declared in Constructor */
// 	double maxCarrotHoriz = maxCarrotHorizDefault ;
// 	double maxCarrotVerticalUP  = maxCarrotVertDefaultUP ;
// 	double maxCarrotVerticalDOWN  = maxCarrotVertDefaultDOWN ;
	//flightMode = flightMode[flightModeIndex] ;
}
/* Deconstructor */
FlightControl::~FlightControl() {
}


/* 
   Initializes something
*/
void FlightControl::initFlight()
{

}
/* 	ComputeXYZSetpoints() AKA where it all happens
   	> determines setpoints as a function of curr location, destination, and flightmode
*/
xyz FlightControl::computeXYZSetpoints(xyz fdest, xyz curr_loc, int flightMode, double yaw, xyz IP)
{ 
    xyz xyzSP, carrotVecXY, temp ;
    double carrotZ ;
    switch(flightMode){
        case CHARGE :
            break ;
        case TAKEOFF :
            KiZ = 0.0; KpZ = 0.0; KpdZ = KPDZ; // ************************************
            carrotZ = computeCarrotVectorZ(fdest, curr_loc, maxCarrotVertUP, flightMode); //***************
            xyzSP = rotateVector(fdest,curr_loc,yaw) ;
            xyzSP.z = carrotZ ;
//            if (carrotZ > fdest.z){flightModeIndex++;}
			if(abs(fdest.z-curr_loc.z) <= 0.1){flightModeIndex++;}
            break ;
        case TRANSLATE :
            KiN = 0.0; KiE = 0.0; KiZ = KIZ ;
            carrotVecXY = computeCarrotVectorXY(fdest, curr_loc, IP, maxCarrotHoriz) ;
            xyzSP = rotateVector(carrotVecXY, curr_loc, yaw)  ;
            xyzSP.z = fdest.z ;
            temp.x = fdest.x - curr_loc.x ;
            temp.y = fdest.y - curr_loc.y ;
            if (computeMagnitude(carrotVecXY, TRUE) > computeMagnitude(temp, TRUE)) {flightModeIndex++;}
            break ;
        case LAND :
            KpZ = 0.0 ; KiN = KIN ; KiE = KIE ; KpdZ = KPDZ ;// ************************************
            carrotZ = computeCarrotVectorZ(fdest,curr_loc, maxCarrotVertDOWN,flightMode); //***************
            xyzSP = rotateVector(fdest,curr_loc,yaw) ;
            xyzSP.z = curr_loc.z - carrotZ ;
            if (abs(curr_loc.z-IP.z) < 0.04 ){flightModeIndex++;}
            break ;
        case HOVER :
            KpZ = KPZ; KiZ = KIZ ; KiN = KIN ; KiE = KIE ; KpdZ = 0.0 ;
            xyzSP = rotateVector(fdest, curr_loc, yaw) ;
            xyzSP.z = fdest.z ;
            break ;
    }
    return xyzSP ;
}
// THIS FUNCTION NEEDS TO BE LOOKED AT
double FlightControl::computeCarrotVectorZ(xyz fdest, xyz curr_loc, double maxCarrotVert, int flightMode){
    double dist, zSP ;
    dist = fdest.z - curr_loc.z ;
    if (flightMode == TAKEOFF) {
    	if (abs(dist) <= maxCarrotVertDefaultUP ) {
    		zSP = fdest.z ;
//    		zSP = 1.0 ;
		}
    	else {
    		zSP = curr_loc.z + maxCarrotVertDefaultUP ;
//    		zSP = 1.5;
		}
	}
	else {
		if ( abs(dist) <= maxCarrotVertDefaultDOWN )
    		zSP = fdest.z ;
    	else
    		zSP = curr_loc.z - maxCarrotVertDefaultDOWN ;
	}
    return zSP ;
}


/* computeCarrotVectorXY()
 * 5/4/16 >> Added logic for the initial position to not be at (0,0). 
 * > dest = ACTUAL ENU FINAL destination coors relative to antenna
 * > curr_loc = ACTUAL ENU current coors relative to antenna
 * > initial_loc = ACTUAL ENU initial coors relative to antenna
 * > maxCarrot = magnitude of Carrot vector (variable)
 * RETURNS Setpoints for E/N Coordinates
*/
xyz FlightControl::computeCarrotVectorXY(xyz dest, xyz curr_loc, xyz initial_loc, double maxCarrotHoriz){
    double Amag, Bmag, Pmag, QnMag, Cmag, Hmag ;
    xyz A, B, Qn, H, xySetpoint ;
    Cmag = maxCarrotHoriz ;
    Qn.x = curr_loc.x - initial_loc.x ;
    Qn.y = curr_loc.y - initial_loc.y ;
    QnMag = computeMagnitude(Qn, TRUE) ;
    H.x = dest.x - initial_loc.x ;
    H.y = dest.y - initial_loc.y ;
    Hmag = computeMagnitude(H, TRUE) ;
    A.x = H.x*(computeDotProduct(Qn, H, TRUE))/pow(Hmag,2) ; // Project Qn onto H
    A.y = H.y*(computeDotProduct(Qn, H, TRUE))/pow(Hmag,2) ;
    Amag = computeMagnitude(A, TRUE);
    Bmag = sqrt(pow(QnMag,2) - pow(Amag,2)) ;
    if (Cmag > Bmag){Cmag = Bmag;}
    Pmag = sqrt(pow(Cmag,2) - pow(Bmag,2));
    
    xySetpoint.x = initial_loc.x + (Pmag + Amag)*H.x/pow(Hmag,2) ;
    xySetpoint.y = initial_loc.y + (Pmag + Amag)*H.y/pow(Hmag,2) ;
    return xySetpoint ;
}



/* rotateVector() fixes roll/pitch setpoint problem 
   > After this function runs, N_input and E_input to N/E PID must be 
     zero
*/
xyz FlightControl::rotateVector(xyz Target, xyz Actual, double yaw) {
    xyz Error, ErrorT ;
    // Find 2d Error Vector
    Error.x = Target.x - Actual.x ; // EAST ERROR
    Error.y = Target.y - Actual.y ; // NORTH ERROR
    //Rotate Error Vector by Negative Yaw
    ErrorT.x = Error.x*cos(-deg2rad*yaw) - Error.y*sin(-deg2rad*yaw) ; // Ex*cos(-yaw) - Ey*cos(-yaw) Rotated East Comp.
    ErrorT.y = Error.y*sin(-deg2rad*yaw) + Error.x*cos(-deg2rad*yaw) ; // Ey*sin(-yaw) + Ex*cos(-yaw) Rotated North Comp.
    return ErrorT ; // Esetpoint = ErrorT.x, NSetpoint = ErrorT.y
}
/* rotateAxes() fixes the boundary problem as yaw actual cross the zero to 360 degreee border in both directions
*/
double FlightControl::rotateAxes(double yawSP, double yaw) {
    double diff, yawT ;
    diff = yawSP - 180.0 ;
    yawT = yaw - diff ;
    if (yawT < 0.0){ yawT = yawT + 360.0;}
    else if (yawT > 359.9999){yawT = yawT - 360.0;}
    return yawT ;
}


/* computeGroundSpeed() computes the magnitude of (x,y) velocity vector */
double FlightControl::computeGroundSpeed(xyz curr_loc, xyz last_loc, double lastTime) {
    double dVx, dVy, dt ;
    dt = (millis() - lastTime)/1000.0 ;
    dVx = abs(curr_loc.x - last_loc.x)/dt ;
    dVy = abs(curr_loc.y - last_loc.y)/dt ;
    return sqrt(pow(dVx,2)+pow(dVy,2)) ;
}

/* computeVerticalSpeed() computes the magnitude of z velocity vector */
double FlightControl::computeVerticalSpeed(xyz curr_loc, xyz last_loc, double lastTime) {
    double dVz, dt ;
    dt = (millis() - lastTime)/1000.0 ;
    dVz = (curr_loc.z - last_loc.z)/dt ;
    return dVz ;
}

/* computeMagnitude() computes magnitude of vector xyz, if xyOnly == 0 computes 3d */
double FlightControl::computeMagnitude(xyz vec, int xyOnly){
	if( xyOnly == FALSE )
		return sqrt(pow(vec.x,2)+pow(vec.y,2)+pow(vec.z,2)) ;
	else
		return sqrt(pow(vec.x,2)+pow(vec.y,2)) ;
}

double FlightControl::computeDotProduct(xyz vec1, xyz vec2, int xyOnly){
	if ( xyOnly == FALSE )
		return vec1.x*vec2.x+vec1.y*vec2.y+vec1.z*vec2.z ;
	else
		return vec1.x*vec2.x+vec1.y*vec2.y ;
}
	
xyz FlightControl::LLH_to_XYZ(xyz llh, double cs_height) {
	double LAT, LON ;
	xyz XYZ ;
	LAT = llh.x*PI/180.0 ;
	LON = llh.y*PI/180.0 ;
	XYZ.x = -cs_height*cos(LAT)*cos(LON) ;
	XYZ.y = cs_height*sin(LAT) ;
	XYZ.z = cs_height*cos(LAT)*sin(LON) ;
	return XYZ ;
}

void setInitialPosition (xyz initialPosition){
	
}

