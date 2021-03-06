#include <gps_example/GpsConversion.h>

// Empty constructor and destructor methods
gps_example::GpsConversion::GpsConversion() {
}

gps_example::GpsConversion::~GpsConversion() {
}

/* This function is used to convert geodetic coordinates to ENU relative
   to the reference coordinates */

void gps_example::GpsConversion::setCurrentPosition(double latitude,
		double longitude, double altitude) {

  // Intermediate ECEF coordinates
	double ecef_x;
	double ecef_y;
	double ecef_z;

  // Convert latitude and longitude to radians
	double lat_rad = M_PI / 180.0 * latitude;
	double lon_rad = M_PI / 180.0 * longitude;

  // Convert input geodetic coordinates to ECEF
	geodeticToEcef(lat_rad, lon_rad, altitude, ecef_x, ecef_y, ecef_z);

  // Convert intermediate ECEF to ENU
	ecefToEnu(ecef_x, ecef_y, ecef_z);

}

/* This function sets the reference coordinates to which all other
   GPS coordinates will be converted to ENU */
void gps_example::GpsConversion::setRefCoordinates(double latitude,
		double longitude, double altitude) {

  /* Set reference geodetic coordinates, with latitude and longitude
     converted to radians */
	ref_lat = M_PI / 180.0 * latitude;
	ref_lon = M_PI / 180.0 * longitude;
	ref_alt = altitude;

  // Convert reference geodetic coordinates to reference ECEF coordinates
	geodeticToEcef(ref_lat, ref_lon, ref_alt, ref_ecef_x, ref_ecef_y,
			ref_ecef_z);

}

/* This function returns the converted ENU coordinate output */
void gps_example::GpsConversion::getEnu(double& enu_x, double& enu_y,
		double& enu_z) {

	enu_x = enu_x_;
	enu_y = enu_y_;
	enu_z = enu_z_;

}

#define A	6378137           // Earth's semi-major axis length
#define E2	6.6943799014e-3 // The square of Earth's eccentricity

/* This function applies the conversion equation from geodetic to ECEF */
void gps_example::GpsConversion::geodeticToEcef(double lat, double lon,
		double alt, double& ecef_x, double& ecef_y, double& ecef_z) {

  // Distance from ECEF z axis to the surface
	double N = A / sqrt(1 - E2 * sin(lat) * sin(lat));

  // Compute equivalent ECEF coordinates
	ecef_x = (N + alt) * cos(lat) * cos(lon);
	ecef_y = (N + alt) * cos(lat) * sin(lon);
	ecef_z = (N * (1 - E2) + alt) * sin(lat);
}

/* This function applies the coordinate transformation from ECEF to ENU */
void gps_example::GpsConversion::ecefToEnu(double ecef_x, double ecef_y,
		double ecef_z) {

  // Intermediate variables to hold the offset ECEF coordinates
	double dx = ecef_x - ref_ecef_x;
	double dy = ecef_y - ref_ecef_y;
	double dz = ecef_z - ref_ecef_z;

  // Apply rotation matrix
	double c_lat = cos(ref_lat);
	double s_lat = sin(ref_lat);
	double c_lon = cos(ref_lon);
	double s_lon = sin(ref_lon);

	enu_x_ = -s_lon * dx + c_lon * dy;
	enu_y_ = -s_lat * c_lon * dx - s_lat * s_lon * dy + c_lat * dz;
	enu_z_ = c_lat * c_lon * dx + c_lat * s_lon * dy + s_lat * dz;

}
