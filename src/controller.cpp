#include <main.h>
#include <ArduinoEigenDense.h>

using namespace Eigen;

void controllerSetup()
{
}

void controllerUpdate()
{
  static unsigned long lastQueryTime = 0;
  unsigned long now = millis();
  if (now - lastQueryTime >= MSP_UPDATE_INTERVAL_MSEC)
  {
    lastQueryTime = now;

    Matrix3d m = Matrix3d::Random();
    m = (m + Matrix3d::Constant(1.2)) * 50;
    Vector3d v(1, 2, 3);
    Vector3d vo = m * v;
    Serial.println(vo(0));
  }
}