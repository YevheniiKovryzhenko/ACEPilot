/**
 * @file coordinates.c
 *
 **/

#include <coordinates.h>
#include <stdio.h>

origin_t origin;

# define ERP 6356.752       //Polar radius of Earth in km
# define ERE 6378.137      //Equatorial radius of Earth in km
# define D2R M_PI/180       //degrees to radians conversion

/* Geocentric radius calculator */
double __get_GR__(double lat)
{
    double R = sqrt(
        (pow((pow(ERE, 2) * cos(lat * D2R)), 2) + pow((pow(ERP, 2) * sin(lat * D2R)), 2)) 
        / 
        (pow((ERE * cos(lat * D2R)), 2) + pow((ERP * sin(lat * D2R)), 2))); //Geocentric radius in km
    return R*1E3;
}

int coordinates_init()
{
    lla_t origin_lla = {0, 0, 0};
    origin.lla = origin_lla;
    origin.ecef = lla2ecef(&origin.lla);

    // Set origin as uninitialized until good gps data is received
    origin.initialized = 0;

    return 0;
}

ned_waypoint_t lla2ned(const lla_t* lla)
{
    ecef_waypoint_t p = lla2ecef(lla);
    p.x -= origin.ecef.x;
    p.y -= origin.ecef.y;
    p.z -= origin.ecef.z;

    const double theta = (-origin.lla.lat - 90) * D2R;
    const double phi = origin.lla.lon * D2R;

    ned_waypoint_t pned;
    pned.x = (p.x * cos(phi) + p.y * sin(phi)) * cos(theta) - p.z * sin(theta);
    pned.y = -p.x * sin(phi) + p.y * cos(phi);
    pned.z = (p.x * cos(phi) + p.y * sin(phi)) * sin(theta) + p.z * cos(theta);

    return pned;
}

ecef_waypoint_t lla2ecef(const lla_t* lla)
{
    ecef_waypoint_t p;
    const double re = __get_GR__(lla->lat) + lla->alt;
    const double phi = lla->lon * D2R;
    const double theta = lla->lat * D2R;

    p.x = re * cos(theta) * cos(phi);
    p.y = re * cos(theta) * sin(phi);
    p.z = re * sin(theta);

    return p;
}

void set_origin(const lla_t* origin_lla)
{
    origin.lla = *origin_lla;
    origin.ecef = lla2ecef(origin_lla);
}