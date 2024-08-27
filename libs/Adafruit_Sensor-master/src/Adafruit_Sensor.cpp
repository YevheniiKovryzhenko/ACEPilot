#include "Adafruit_Sensor.hpp"
#include <cstdio> //for printf

/**************************************************************************/
/*!
    @brief  Prints sensor information to serial console
*/
/**************************************************************************/
void Adafruit_Sensor::printSensorDetails(void) {
  sensor_t sensor;
  //getSensor(&sensor);
  printf("------------------------------------\n");
  printf("Sensor:       ");
  printf("%s\n",sensor.name);
  printf("Type:         ");
  switch ((sensors_type_t)sensor.type) {
  case SENSOR_TYPE_ACCELEROMETER:
    printf("Acceleration (m/s2)\n");
    break;
  case SENSOR_TYPE_MAGNETIC_FIELD:
      printf("Magnetic (uT)\n");
    break;
  case SENSOR_TYPE_ORIENTATION:
      printf("Orientation (degrees)\n");
    break;
  case SENSOR_TYPE_GYROSCOPE:
    printf("Gyroscopic (rad/s)\n");
    break;
  case SENSOR_TYPE_LIGHT:
    printf("Light (lux)\n");
    break;
  case SENSOR_TYPE_PRESSURE:
    printf("Pressure (hPa)\n");
    break;
  case SENSOR_TYPE_PROXIMITY:
    printf("Distance (cm)\n");
    break;
  case SENSOR_TYPE_GRAVITY:
    printf("Gravity (m/s2)\n");
    break;
  case SENSOR_TYPE_LINEAR_ACCELERATION:
    printf("Linear Acceleration (m/s2)\n");
    break;
  case SENSOR_TYPE_ROTATION_VECTOR:
    printf("Rotation vector\n");
    break;
  case SENSOR_TYPE_RELATIVE_HUMIDITY:
    printf("Relative Humidity (%)\n");
    break;
  case SENSOR_TYPE_AMBIENT_TEMPERATURE:
    printf("Ambient Temp (C)\n");
    break;
  case SENSOR_TYPE_OBJECT_TEMPERATURE:
    printf("Object Temp (C)\n");
    break;
  case SENSOR_TYPE_VOLTAGE:
    printf("Voltage (V)\n");
    break;
  case SENSOR_TYPE_CURRENT:
    printf("Current (mA)\n");
    break;
  case SENSOR_TYPE_COLOR:
    printf("Color (RGBA)\n");
    break;
  case SENSOR_TYPE_TVOC:
    printf("Total Volatile Organic Compounds (ppb)\n");
    break;
  }
  printf("Driver Ver:   ");
  printf("%s\n",sensor.version);
  printf("Unique ID:    ");
  printf("%s\n",sensor.sensor_id);
  printf("Min Value:    ");
  printf("%s\n",sensor.min_value);
  printf("Max Value:    ");
  printf("%s\n",sensor.max_value);
  printf("Resolution:   ");
  printf("%s\n",sensor.resolution);
  printf("------------------------------------\n\n");
}
