/*
  FlightControl.cpp
  Written by Brad Danielson, Suzanne Reisberg, and Rudy Hulse
  Controls all aspects of flight
*/

#include "FlightControl.h"

/* Constructor */
FlightControl::FlightControl() {
    /* Initial Values Declared in Constructor */
	double maxCarrotHoriz = maxCarrotHorizDefault ;
	double maxCarrotVerticalUP  = maxCarrotVertDefaultUP ;
	double maxCarrotVerticalDOWN  = maxCarrotVertDefaultDOWN ;
	flightMode = flightMode[flightModeIndex] ;
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
xyz FlightControl::computeXYZSetpoints(xyz fdest, xyz curr_loc, int flightMode, double yaw, xyz initialPosition)
{ 
    xyz xyzSP, xyz carrotVecXY, xyz temp ;
    double carrotZ ;
    switch(flightMode){
        case CHARGE :
            // CHARGE LOGIC **************************
            break ;
        case TAKEOFF :
            KiZ = 0.0 ; // ************************************
            carrotZ = computeCarrotVectorZ(fdest, curr_loc, maxCarrotVert, flightMode); //***************
            xyzSP = {fdest.x, fdest.y,carrotZ};
            if (carrotZ > fdest.z - curr_loc.z){flightModeIndex++;}
            break ;
        case TRANSLATE :
            KiN = 0.0; KiE = 0.0; KiZ = KIZ ;// ********************************
            // TRANSLATE LOGIC *********************** xyz dest = computeCarrotvectorXY(); xyz dest = rotateVector(yaw)
            carrotVecXY = computeCarrotVectorXY(fdest, curr_loc, intialPosition, maxCarrotHoriz) ;
            xyzSP = rotateVector(carrotVecXY, curr_loc, yaw)  ;
            xyzSP.z = fdest.z ;
            temp.x = fdest.x - curr_loc.x ;
            temp.y = fdest.y - curr_loc.y ;
            if (computeMagnitude(carrotVecXY, TRUE) > computeMagnitude(temp, TRUE)) {flightModeIndex++;}
            break ;
        case LAND :
            KiZ = 0.0 ; KiN = KIN ; KiE = KIE ; // ************************************
            // LANDING LOGIC ****************************
            carrotZ = computeCarrotVectorZ(fdest,curr_loc, maxCarrotVert,flightMode); //***************
            xyzSP.x = fdest.x ; //{fdest.x, fdest.y,carrotZ};
            xyzSP.y = fdest.y ;
            xyzSP.z = carrotZ ;
            if (abs(curr_loc.z-initialPosition.z) < 0.04 ){flightModeIndex++;}
            break ;
        case HOVER :
            KiZ = KIZ ; KiN = KIN ; KiE = KIE ;
            xyzSP.x = flightcoors.x[flightModeIndex] ;
            xyzSP.y = flightcoors.y[flightModeIndex] ;
            xyzSP.z = flightcoors.z[flightModeIndex] ;
            break ;
    }
    return xyzSP ;
}

double FlightControl::computeCarrotVectorZ(xyz fdest, xyz curr_loc, double maxCarrotVert, int flightMode){
    double dist, zSP ;
    dist = fdest.z - curr_loc.z ;
    if (abs(dist) < maxCarrotVert )
        zSP = fdest.z ;
    else {
        if (flightMode == TAKEOFF)
            zSP = curr_loc.z + maxCarrotVertUP ;
        else if (flightMode == LANDING)
            zSP = curr_loc.z - maxCarrotVertDOWN ;
    }
    return zSP ;
}

// xyz FlightControl::computeCarrotVectorXY(xyz dest, xyz curr_loc, double maxCarrotHoriz){
// 	double Qmag, Dmag, Amag, Bmag, Xmag, QDmag, Cmag = maxCarrotHoriz ; // See carrotVector_BRAD.m for ref
// 	xyz A, B, xySetpoint ;
// 	Dmag = computeMagnitude(dest, TRUE) ;
// 	Qmag = computeMagnitude(curr_loc, TRUE) ;
// 	B.x = (computeDotProduct(dest, curr_loc, TRUE)/pow(Dmag,2))*dest.x ;
// 	B.y = (computeDotProduct(dest, curr_loc, TRUE)/pow(Dmag,2))*dest.y ;
// 	Bmag = computeMagnitude(B, TRUE);
// 	A.x = curr_loc.x - B.x ;
// 	A.y = curr_loc.y - B.y ;
// 	Amag = computeMagnitude(A, TRUE) ;
// 	QDmag = sqrt(pow(curr_loc.x-dest.x,2)+pow(curr_loc.y-dest.y,2)); // Actual Distance from Destination
// 	if ( Cmag > QDmag ) {// If QC is closer to Dest than end of CarrotVector, Hover
// 		Cmag = QDmag ;
// 		flightMode = HOVER ;
// 	}
// 	else if ( Amag > Cmag ) { 
// 		Cmag = Amag ;
// 	}
// 	Xmag = sqrt(pow(Cmag, 2)-pow(Amag,2));
// 	xySetpoint.x = ((Bmag+Xmag)/Dmag)*dest.x ;
// 	xySetpoint.x = ((Bmag+Xmag)/Dmag)*dest.y ;
// 	return xySetpoint ;
// }

/* computeCarrotVectorXY()
 * 5/4/16 >> Added logic for the initial position to not be at (0,0). 
 * > dest = ACTUAL ENU FINAL destination coors relative to antenna
 * > curr_loc = ACTUAL ENU current coors relative to antenna
 * > initial_loc = ACTUAL ENU initial coors relative to antenna
 * > maxCarrot = magnitude of Carrot vector (variable)
 * RETURNS Setpoints for E/N Coordinates
*/
xyz FlightControl::computeCarrotVectorXY(xyz dest, xyz curr_loc, xyz initial_loc, double maxCarrotHoriz){
    double Amag, Bmag, Pmag, Qnmag, Cmag, Hmag ;
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
    if (Cmag < Bmag){Cmag = Bmag;}
    Pmag = sqrt(pow(Cmag,2) - pow(Bmag,2));
    
    xySetpoint.x = intial_loc.x + (Pmag + Amag)*H.x/pow(Hmag,2) ;
    xySetpoint.y = intial_loc.y + (Pmag + Amag)*H.y/pow(Hmag,2) ;
    return xySetpoint ;
}



/* rotateVector() fixes roll/pitch setpoint problem 
   > After this function runs, N_input and E_input to N/E PID must be 
     zero
*/
xyz FlightControl::rotateVector(xyz Target, xyz Actual, double yaw) {
    xyz Error, ErrorT ;
    // Find 2d Error Vector
    Error.x = Target.x - Actual.x ; 
    Error.y = Target.y - Actual.y ;
    //Rotate Error Vector by Yaw
    ErrorT.x = -(Error.y*sin(deg2rad*yaw) + Error.x*cos(deg2rad*yaw)) ;
    ErrorT.y = -(Error.y*cos(deg2rad*yaw) - Error.x*sin(deg2rad*yaw)) ;
    return ErrorT ; // Esetpoint = ErrorT.y, NSetpoint = ErrorT.x
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
    dt = (millis() - lastTime)*1000.0 ;
    dVx = abs(curr_loc.x - last_loc.x)/dt ;
    dVy = abs(curr_loc.y - last_loc.y)/dt ;
    return sqrt(pow(dVx,2)+pow(dVy,2)) ;
}

/* computeVerticalSpeed() computes the magnitude of z velocity vector */
double FlightControl::computeVerticalSpeed(xyz curr_loc, xyz last_loc, double lastTime) {
    double dVz, dt ;
    dt = (millis() - lastTime)*1000.0 ;
    dVz = abs(curr_loc.z - last_loc.z)/dt ;
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

