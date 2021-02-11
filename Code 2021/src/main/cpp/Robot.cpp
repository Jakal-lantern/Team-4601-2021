// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Team 4601 2021 Code

#pragma region // References

#include "Robot.h"

// Standard References
#include <iostream>
#include <memory>
#include <unistd.h>
#include <string>
#include <math.h>

// 'frc/' References
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/WPILib.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/IterativeRobot.h>
#include <frc/Joystick.h>
#include <frc/PWMVictorSPX.h>
#include <frc/PWM.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/util/Color.h>
#include <frc/Timer.h>

// Misc References
#include <cameraserver/CameraServer.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <adi/ADIS16470_IMU.h>

#pragma endregion

using namespace frc;

class Robot : public TimedRobot 
{
  
  #pragma region // Initialization

  // Auton
  LiveWindow *lw = LiveWindow::GetInstance();
  frc::Timer *timer = new Timer();
  int autonSwitch;
  bool autonIsActive;

  // Motor Controller
  PWMVictorSPX *light = new PWMVictorSPX(0);
  PWMVictorSPX *fRight = new PWMVictorSPX(1);
  PWMVictorSPX *fLeft = new PWMVictorSPX(2);
  PWMVictorSPX *turret = new PWMVictorSPX(3);
  PWMVictorSPX *arm = new PWMVictorSPX(4);
  PWMVictorSPX *turretShoot = new PWMVictorSPX(5);

  // Joystick
  DifferentialDrive m_robotDrive{*fRight, *fLeft};
  Joystick m_stick{0};
  Joystick m_stick2{1};
  XboxController controller1{2};

  // Camera
  cs::UsbCamera camera1 = CameraServer::GetInstance()->StartAutomaticCapture(0);

  // Variables
  std::string colorString;
  bool canElevate = true;
  bool driveState = true;
  float steeringAdjust;
  float controlConstant = -0.1f;
  double throttle1;
  double pi = 4.0*atan(1.0);
  double g = 9.8;
  double driveCommand;
  double kP = 0.005;
  double kAngleSetpoint = 0.0;

  // Digital Input
  frc::DigitalInput lSwitch1{12};
  frc::DigitalInput lSwitch2{14};

  // Digital Output
  frc::DigitalOutput LED1{10};

  // Sensor
  frc::ADIS16470_IMU imu{frc::ADIS16470_IMU::IMUAxis::kZ, frc::SPI::Port::kOnboardCS0, frc::ADIS16470CalibrationTime::_4s};

  // Limelight Variables
  std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  double clamp (double in, double minval, double maxval)
  {
    if (in > maxval) return maxval;
    if (in < maxval) return maxval;
    return in;
  }
  double LimelightTurnCmd;
  double LimelightDriveCmd;
  bool LimelightHasTarget;

  #pragma endregion

  // TODO: Revisit auton section to use gyro
  #pragma region // Auton Functions

  // TODO: Research what to do with 'RobotInit' function
  void RobotInit()
  {
  }

  // Runs before auton, cannot initialize values inside it
  void AutonomousInit()
  {
    imu.Calibrate();
    timer->Start();
    autonIsActive = true;
  }

  // Auton Main Function
  void AutonomousPeriodic()
  {
    /*
    * Switch behaviour based on starting position
    * 0 = right
    * 1 = middle
    * 2 = left
    */

    switch (lSwitch1.Get())
    {
      case (0):
        autonSwitch = 0;
        break;

      case (1):
      if (lSwitch2.Get() == 0)
      {
        autonSwitch = 1;
      }
      else 
      {
        autonSwitch = 2;
      }
      break;
    }
  }

  #pragma endregion

  #pragma region  // Main Robot Function

  void TeleopPeriodic()
  {
    // TODO: Find a better way to tell if auton is active
    autonIsActive = false;  // Do I need this boolean?

    #pragma region // Pilot Controls

    // Drive with arcade style
    // m_robotDrive.ArcadeDrive(m_stick.GetY(), m_stick.GetX());

    // Drive with tank style
    // m_robotDrive.TankDrive(m_stick2.GetY(), m_stick.GetY());

    // Is 'GetRawButton' best way of getting input?
    if (m_stick2.GetRawButton(13))
    {
      driveState = true;
    }

    if (m_stick2.GetRawButton(14))
    {
      driveState = false;
    }

    if (driveState)
    {
      // TODO: Implement 'UpdateLimelightTracking' function
      // UpdateLimelightTracking();

      if (controller1.GetAButton())
      {
        if (LimelightHasTarget)
        {
          m_robotDrive.ArcadeDrive(LimelightDriveCmd, LimelightTurnCmd);
        }
        else
        {
          m_robotDrive.ArcadeDrive(0.0, 0.0);
        }
      }
      else
      {
        double fwd = m_stick.GetY();
        double turn = m_stick.GetX();
        throttle1 = m_stick.GetThrottle();

        if (throttle1 > 0.5)
        {
          m_robotDrive.ArcadeDrive(fwd, -turn);
        }
        else if (throttle1 < -0.5)
        {
          m_robotDrive.ArcadeDrive(-fwd, turn);
        }
        else 
        {
          m_robotDrive.ArcadeDrive(-fwd / 2, turn / 1.7);
        }
      }
    }
    else
    {
      // Labeled as tank auton code?
      // Possible answer: Auton code placed in main telop function for testing
      // TODO: Test
      float currentDistance = EstimateDistance(); // TODO: Implement 'EstimateDistance' function
      float desiredDistance = 500;
      int distanceThresh = 5;

      if (controller1.GetBButton())
      {
        float drivingDistance = desiredDistance - currentDistance;

        if (drivingDistance < distanceThresh || drivingDistance > -distanceThresh)
        {
          driveCommand = 0;
        }
        if (drivingDistance < desiredDistance)
        {
          driveCommand = 0.5;
        }
        if (drivingDistance > desiredDistance)
        {
          driveCommand = -0.5;
        }

        m_robotDrive.TankDrive(driveCommand, driveCommand);
      }
    }


  }


  



};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return StartRobot<Robot>();
}
#endif

#pragma region // Maps And Assignments

/*

  RoboRIO Map
  +-------------------------------------+
  | LED ligth strip = 0
  | Right Drive Cim = 1
  | Left Drive Cim  = 2
  | Turret          = 3
  | Arm             = 4
  | TurretShoot     = 5
  | 
  |
  | Color Sensor = I2C Port
  +-------------------------------------+

RoboRIO Digital Pins Map
  +-------------------------------------+
  | Auton Switch 1 (IN) = D15/D16(gnd)
  | Auton Switch 2 (IN) = D19/D20(gnd)
  | LED1 (OUT) = D11/D12(gnd)
  +-------------------------------------+

  USB Map
  +-------------------------------------+
  | Right Joystick  = 0
  | Left Joystick   = 1
  | Xbox Controller = 2
  +-------------------------------------+
  
  Xbox Controller Map
  +-------------------------------------+
  | A = Spin To Green
  | B = Spin To Red
  | Y = Spin To Yellow
  | X = Spin To Blue
  |
  | L1 = Outtake
  | L2 = Upspeed On Shooter
  | L3 =
  | R1 = Intake
  | R2 = Enable Shooting / Conveyor
  | R3 =
  |
  | DPad Right = Elevator Up
  | DPad Left  =
  | DPad Up    =
  | DPad Down  = Elevator Down
  |
  | Left Joystick  =
  | Right Joystick = x - Spin Turret
  |                  y - Adjust Angle
  | 
  | Start  =
  | Select =
  +-------------------------------------+

  Joystick Map
  +-------------------------------------+
  | Right Joystick = Arcade Movement
  | Left Joystick  = 
  |
  | RIGHT STICK
  | Right Trigger     = Switch Arcade Movement
  | On-Stick Button L = Arm Position 1
  | On-Stick Button R = Arm Position 2
  | On-Stick Button M = Limelight Lock On
  |
  | Left Button 5  = 
  | Left Button 6  = 
  | Left Button 7  =
  | Left Button 8  = Limelight Zoom Default
  | Left Button 9  = Limelight Zoom 1
  | Left Button 10 = Limelight Zoom 2
  +-------------------------------------+
  
*/

#pragma endregion