#ifndef GPSCONVERSION_H_
#define GPSCONVERSION_H_

#include <math.h>

namespace gps_example {

class GpsConversion {
public:
	GpsConversion(); // Constructor
	~GpsConversion(); // Destructor

  /* Prototypes for public methods */
	void setCurrentPosition(double latitude, double longitude, double altitude);
	void setRefCoordinates(double latitude, double longitude, double altitude);

	void getEnu(double& enu_x, double& enu_y, double& enu_z);

private:

  /* Prototypes for private methods */
	void geodeticToEcef(double lat, double lon, double alt, double& ecef_x,
			double& ecef_y, double& ecef_z);

	void ecefToEnu(double ecef_x, double ecef_y, double ecef_z);

  /* Private properties to store reference and output values */
	double ref_lat;
	double ref_lon;
	double ref_alt;

	double ref_ecef_x;
	double ref_ecef_y;
	double ref_ecef_z;

	double enu_x_;
	double enu_y_;
	double enu_z_;

};

}

#endif /* GPSCONVERSION_H_ */
