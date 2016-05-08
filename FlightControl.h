#ifndef FLIGHTCONTROL_H_
#define FLIGHTCONTROL_H_

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <math.h>

/* Definitions*/
#define maxCarrotHorizDefault 0.5 // Max Magnitude of Horizontal Heading Vector
#define maxCarrotVertDefaultUP 0.5
#define maxCarrotVertDefaultDOWN 0.03 ;
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

// flightsequence = {takeoff, hover, translate, hover, translate, hover, translate, hover, translate, land}
// flightseqxyz = {xyz takeoff, xyz sitthere, xyz destination1, xyz sithere1, xyz backhome, xyz chargingpad}

struct xyz {
	double x ; // N = x+
	double y ; // E = y+
	double z ;
} ;

struct xyzf {
    double x ;
    double y ;
    double z ;
    int fix ;
} ;


class FlightControl {
public:
	FlightControl();
	virtual ~FlightControl();
	void initFlight();
	xyz computeXYZSetpoints(xyz fdest, xyz curr_loc, int flightMode, double yaw, xyz initialPosition);
	xyz computeCarrotVectorXY(xyz dest, xyz curr_loc) ;
	xyz rotateVector(xyz Target, xyz Actual, double yaw) ;
	double computeCarrotVectorZ(xyz fdest, xyz curr_loc, double maxCarrotVert, int flightMode);
	double rotateAxes(double yawSP, double yaw) ;
	double computeGroundSpeed(xyz curr_loc, xyz last_loc, double lastTime) ;
	double computeVerticalSpeed(xyz curr_loc, xyz last_loc, double lastTime) ;
private:
	xyz last_loc;
	double maxCarrotHoriz;
	double maxCarrotVertUP;
	double maxCarrotVertDOWN;
	double computeMagnitude(xyz vec, int xyOnly);
	double computeDotProduct(xyz vec1, xyz vec2, int xyOnly);

};

#endif /* FLIGHTCONTROL_H_ */
