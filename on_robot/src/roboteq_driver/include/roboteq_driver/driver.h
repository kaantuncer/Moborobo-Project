#include <ros/ros.h>
#include <serial/serial.h>

#include <sstream>

#include "Constants.h"
#include "ErrorCodes.h"
#include "RoboteqDevice.h"

using namespace std;

class Driver
{
public:
  Driver()
  {
    Connect();
    Arm();
  }
  ~Driver() { Disconnect(); }

  int Connect()
  {
    string response = "";
    int status = device.Connect("/dev/ttyACM0");
    if (status != RQ_SUCCESS)
    {
      ROS_INFO("Failed to connect with motor driver");
    }
    else
    {
      ROS_INFO("Connection with motor driver has been established");
    }

    return status;
  }
  void Disconnect() { device.Disconnect(); }
  bool IsConnected() { return device.IsConnected(); }
  int Arm()
  {
    int status = 0;
    if ((status = device.SetCommand(_MG, 1)) = RQ_SUCCESS)
      return 0;
    else
      return 1;
  }
  int TurnWheel(int canIndex, int motorIndex, float value)
  {
    int status = 0;
    if ((status = device.SetCanCommand(canIndex, _G, motorIndex, value)) !=
        RQ_SUCCESS)
      return 1;
    else
      return 0;
  }
  int TurnWheelRPM(int motorIndex, int rpm)
  {
    return device.SetCommand(_S, motorIndex, rpm);
  }
  int GetMotorRPM(int motorIndex, int &rpm)
  {
    return device.GetValue(_BS, motorIndex, rpm);
  }
  float GetMotorCurrent(int motorIndex, int &motorCurrent)
  {
    return device.GetValue(_A, motorIndex, motorCurrent);
  }
  bool GetButtonStatus(int button_pin_id) // Diğital pin çıkışları boolean,
                                          // dolayısıyla burası değişecek
  {
    int status = 0, value = 0;
    if ((status = device.GetValue(_DIN, button_pin_id, value)) != RQ_SUCCESS)
      return false;

    else
    {
      if (value == 0)
        return false;
      else
        return true;
    }
  }
  float GetControllerChannel(
      int controller_channel_id) // Diğital pin çıkışları boolean, dolayısıyla
                                 // burası değişecek
  {
    int status = 0, value = 0;
    if ((status = device.GetCanValue(1, _AI, controller_channel_id, value)) !=
        RQ_SUCCESS)
      return -111111;
    else
      return value;
  }

  int GetBatteryVoltage(int voltageIndex, int &voltage)
  {
    return device.GetValue(_V, voltageIndex, voltage);
  }


private:
  RoboteqDevice device;
  void Wait() { sleepms(10); }
};