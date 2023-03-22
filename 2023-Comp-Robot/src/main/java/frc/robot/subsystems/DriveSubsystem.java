// Copyright (c) FIRST and other WPILib contributors. Hi Logan
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveSubsystem extends SubsystemBase {

  // Drive Motors
  public static WPI_TalonFX leftFront = new WPI_TalonFX(4, "rio"); 
  public static WPI_TalonFX leftBack = new WPI_TalonFX(3, "rio");
  public static WPI_TalonFX rightFront = new WPI_TalonFX(2, "rio");
  public static WPI_TalonFX rightBack = new WPI_TalonFX(1, "rio");

  // Gyro
  AHRS gyro = new AHRS(I2C.Port.kMXP);

  // Motor Values 
	final TalonFXInvertType kInvertType = TalonFXInvertType.Clockwise; 
	final NeutralMode kBrakeDurNeutral = NeutralMode.Coast;
  SlewRateLimiter driveFilter = new SlewRateLimiter(1.25);

  // Drive Encoder Units
  final double inchPerRev = (Math.PI*6)/10.71; 
  double driveDistLeft;
  double driveDistRight;

  boolean driveBoolean = false;

  // Auton
  Timer driveTimer = new Timer();
  Timer balanceTimer = new Timer();

  boolean setDist = false;

  boolean speedBoost = true;

  double dist = 0;
  double dist2 = 0;

  public void DriveTeleop() 
  {
    double forward;
    double turn = RobotContainer.XCont.getRightX()*Constants.turnSpeed;
    if (RobotContainer.XCont.getLeftTriggerAxis() >= .5)
    {
      forward = RobotContainer.XCont.getLeftY();
      forward = forward*.2;
      forward = Deadzone(forward);
    }
    else
    {
      forward = RobotContainer.XCont.getLeftY();
      forward = Deadzone(forward);
      forward = driveFilter.calculate(forward);
    }
    turn = Deadzone(turn);
    
    leftFront.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
    leftBack.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
    rightFront.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
    rightBack.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
  }

  public void DriveInit()
  {
    if (driveBoolean == false)
    {
      leftFront.set(ControlMode.PercentOutput, 0);
      leftBack.set(ControlMode.PercentOutput, 0);
      rightFront.set(ControlMode.PercentOutput, 0);
      rightBack.set(ControlMode.PercentOutput, 0);
      leftFront.configFactoryDefault();
      leftBack.configFactoryDefault();
      rightFront.configFactoryDefault();
      rightBack.configFactoryDefault();
      leftFront.setNeutralMode(NeutralMode.Brake);
      leftBack.setNeutralMode(NeutralMode.Brake);
      rightFront.setNeutralMode(NeutralMode.Brake);
      rightBack.setNeutralMode(NeutralMode.Brake);
      leftFront.setInverted(true);
      leftBack.setInverted(true);
      rightFront.setInverted(false); 
      rightBack.setInverted(false);

      TalonFXConfiguration configs = new TalonFXConfiguration();
        
      configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        
      leftFront.configAllSettings(configs);
      leftFront.setSelectedSensorPosition(0);

      rightFront.configAllSettings(configs);
      rightFront.setSelectedSensorPosition(0);
    }
  }
  
  public double Deadzone(double value)
  {
    /* Upper deadzone */
		if (value >= +0.05){
      value = value*Constants.speedLimit;
    return value;}
  
  /* Lower deadzone */
    else if (value <= -0.05){
      value = value*Constants.speedLimit;
    return value;}
  
  /* Outside deadzone */
    else{return 0;}
  }

  public void Balance()
  {
    double pitch = -gyro.getPitch();
    double speed = .09;
    if (pitch > 10)
    {
      leftFront.set(speed);
      leftBack.set(speed);
      rightFront.set(-speed);
      rightBack.set(-speed);
    }
    else if (pitch < -10)
    {
      leftFront.set(-speed);
      leftBack.set(-speed);
      rightFront.set(speed);
      rightBack.set(speed);
    }
    else
    {
      leftFront.set(0);
      leftBack.set(0);
      rightFront.set(0);
      rightBack.set(0);
    }
  }
  public void DriveAutonInit()
  {
    TalonFXConfiguration configs = new TalonFXConfiguration();
			
		configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
			
		leftFront.configAllSettings(configs);
    leftFront.setSelectedSensorPosition(0);

    rightFront.configAllSettings(configs);
    rightFront.setSelectedSensorPosition(0);

    driveTimer.reset();
    driveTimer.start();

    leftFront.setInverted(false);
    leftBack.setInverted(false);
    rightFront.setInverted(false); 
    rightBack.setInverted(false);
  }

  public void DriveAuton()
  {
    driveDistLeft = (leftFront.getSelectedSensorPosition()/2048)*inchPerRev;
    driveDistRight = (rightFront.getSelectedSensorPosition()/2048)*inchPerRev;
    System.out.println("driveDistLeft: "+driveDistLeft);
      if (driveDistLeft < 50 && driveDistRight < 50)
      {
        leftFront.set(.2);
        rightFront.set(-.2);
        leftBack.set(.2);
        rightBack.set(-.2);
      }
      else
      {
        leftFront.set(0);
        rightFront.set(0);
        leftBack.set(0);
        rightBack.set(0);
      }
  }
  public void DriveAutonScore2()
  {
    driveDistLeft = (leftFront.getSelectedSensorPosition()/2048)*inchPerRev;
    driveDistRight = (-rightFront.getSelectedSensorPosition()/2048)*inchPerRev;

    if (driveTimer.get() > 6 && driveTimer.get() <= 15)
    {
      if (driveDistRight < 8)
      {
        leftFront.set(0);
        leftBack.set(0);
        rightBack.set(-0.3);
        rightFront.set(-0.3);
      }
      else if (driveDistRight < 18)
      {
        leftFront.set(.3);
        leftBack.set(.3);
        rightBack.set(-.3);
        rightFront.set(-.3);
      }
      else if (driveDistLeft < 20)
      {
        leftFront.set(.3);
        leftBack.set(.3);
        rightBack.set(0);
        rightFront.set(0);
      }
      else if (driveDistLeft < 150 && driveDistRight < 150)
      {
        leftFront.set(.3);
        leftBack.set(.3);
        rightBack.set(-.3);
        rightFront.set(-.3);
      }
      else
      {
        leftFront.set(0);
        rightFront.set(0);
        leftBack.set(0);
        rightBack.set(0);
      }
    }
  }
  public void DriveAutonScore()
  {
    driveDistLeft = (leftFront.getSelectedSensorPosition()/2048)*inchPerRev;
    driveDistRight = (-rightFront.getSelectedSensorPosition()/2048)*inchPerRev;

    if (driveTimer.get() > 6 && driveTimer.get() <= 15)
    {
      if (driveDistLeft < 8)
      {
        leftFront.set(.3);
        leftBack.set(.3);
        rightBack.set(0);
        rightFront.set(0);
      }
      else if (driveDistLeft < 18)
      {
        leftFront.set(.3);
        leftBack.set(.3);
        rightBack.set(-.3);
        rightFront.set(-.3);
      }
      else if (driveDistRight < 20)
      {
        leftFront.set(0);
        leftBack.set(0);
        rightBack.set(-.3);
        rightFront.set(-.3);
      }
      else if (driveDistLeft < 150 && driveDistRight < 150)
      {
        leftFront.set(.3);
        leftBack.set(.3);
        rightBack.set(-.3);
        rightFront.set(-.3);
      }
      else
      {
        leftFront.set(0);
        rightFront.set(0);
        leftBack.set(0);
        rightBack.set(0);
      }
    }
  }
  public void DriveAutonBalance()
  {
    driveDistLeft = (leftFront.getSelectedSensorPosition()/2048)*inchPerRev;
    driveDistRight = (-rightFront.getSelectedSensorPosition()/2048)*inchPerRev;
    if (driveTimer.get() > 0 && driveDistLeft < 81 && driveTimer.get() < 5)
    {
      leftFront.set(.2);
      leftBack.set(.2);
      rightBack.set(-.2);
      rightFront.set(-.2);
    }
    else if (driveTimer.get() > 5)
    {
      Balance();
    }
    else
    {
      leftFront.set(0);
      leftBack.set(0);
      rightBack.set(0);
      rightFront.set(0);
    }
  }

  public void DriveAutonScoreBalance()
  {
    driveDistLeft = (leftFront.getSelectedSensorPosition()/2048)*inchPerRev;
    driveDistRight = (-rightFront.getSelectedSensorPosition()/2048)*inchPerRev;
    if (driveTimer.get() > 5 && driveDistLeft < 81 && driveTimer.get() < 10)
    {
      leftFront.set(.2);
      leftBack.set(.2);
      rightBack.set(-.2);
      rightFront.set(-.2);
    }
    else if (driveTimer.get() > 10)
    {
      Balance();
    }
    else
    {
      leftFront.set(0);
      leftBack.set(0);
      rightBack.set(0);
      rightFront.set(0);
    }
  }
} 