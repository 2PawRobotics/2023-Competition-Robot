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
  SlewRateLimiter driveFilter = new SlewRateLimiter(0.7);
  SlewRateLimiter turnFilter = new SlewRateLimiter(1.4);

  // Drive Encoder Units
  final double inchPerRev = (Math.PI*6)/10.71; 
  double driveDistLeft;
  double driveDistRight;

  boolean driveBoolean = false;

  // Auton
  Timer driveTimer = new Timer();

  boolean driveSeg1 = false;
  boolean driveSeg2 = false;

  boolean speedBoost = true;

  public void DriveTeleop() 
  {
    double forward = RobotContainer.XCont.getLeftY();
    double turn = RobotContainer.XCont.getRightX()*.7;
    forward = Deadzone(forward);
    turn = Deadzone(turn);
    forward = driveFilter.calculate(forward);
    turn = turnFilter.calculate(turn);
    
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

  public void Balance(){

    double pitch = gyro.getPitch();
    double speed = .095;
    speed = driveFilter.calculate(speed);
    double speed2 = .25;
    speed2 = driveFilter.calculate(speed2);
    if (pitch > 12)
    {
      leftFront.set(speed);
      leftBack.set(speed);
      rightFront.set(speed);
      rightBack.set(speed);
    }
    else if (pitch < 12 && pitch > 5)
    {
      leftFront.set(speed2);
      leftBack.set(speed2);
      rightFront.set(speed2);
      rightBack.set(speed2);
    }
    else if (pitch < -12)
    {
      leftFront.set(-speed);
      leftBack.set(-speed);
      rightFront.set(-speed);
      rightBack.set(-speed);
    }
    else if (pitch > -12 && pitch < -5)
    {
      leftFront.set(-speed2);
      leftBack.set(-speed2);
      rightFront.set(-speed2);
      rightBack.set(-speed2);
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

    driveSeg1 = true;

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
      if (driveDistLeft < 30 && driveDistRight < 40)
      {
        leftFront.set(.1);
        rightFront.set(-.1);
        leftBack.set(.1);
        rightBack.set(-.1);
      }
      else
      {
        leftFront.set(0);
        rightFront.set(0);
        leftBack.set(0);
        rightBack.set(0);
      }
  }

  public void DriveAutonScore()
  {
    driveDistLeft = (leftFront.getSelectedSensorPosition()/2048)*inchPerRev;
    driveDistRight = (rightFront.getSelectedSensorPosition()/2048)*inchPerRev;
    if (driveTimer.get() > 8 && driveTimer.get() <= 15)
    {
      if (driveDistLeft < 30 && driveDistRight < 30)
      {
        leftFront.set(.4);
        rightFront.set(-.4);
        leftBack.set(.4);
        rightBack.set(-.4);
      }
      else
      {
        leftFront.set(0);
        rightFront.set(0);
      }
    }
  }
  public void DriveAutonBalance()
  {
    driveDistLeft = (leftFront.getSelectedSensorPosition()/2048)*inchPerRev;
    driveDistRight = (-rightFront.getSelectedSensorPosition()/2048)*inchPerRev;
    if (driveSeg1 == true)
    {
      if (driveDistLeft < 150 && driveDistRight < 150)
      {
        if (gyro.getPitch() < -1.5)
        {
          leftFront.set(.15);
          rightFront.set(-.15);
          leftBack.set(.15);
          rightBack.set(-.15);
          speedBoost = true;
        }
        else if (gyro.getPitch() >= -1.5 && gyro.getPitch() < 12 && speedBoost == true)
        {
          leftFront.set(.35);
          rightFront.set(-.35);
          leftBack.set(.35);
          rightBack.set(-.35);
        }
        else
        {
          leftFront.set(.2);
          rightFront.set(-.2);
          leftBack.set(.2);
          rightBack.set(-.2);
          speedBoost = false;
        }
      }
      else
      {
        leftFront.set(0);
        rightFront.set(0);
        leftBack.set(0);
        rightBack.set(0);
        driveSeg1 = false;
        driveSeg2 = true;
        System.out.println("drive segment 1 complete");
      }
    }
    else if (driveSeg2 == true)
    {
      System.out.println("drive segment 2 reached");
      System.out.println("driveDistLeft: "+driveDistLeft);
      System.out.println("driveDistRight: "+driveDistRight);
      double speed = .3;
      speed = driveFilter.calculate(speed);
      if (driveDistLeft > 110 && driveDistRight > 110 && gyro.getPitch() > -5)
      {
        leftFront.set(-speed);
        rightFront.set(speed);
        leftBack.set(-speed);
        rightBack.set(speed);
      }
      else
      {
        System.out.println("drive segment 2 complete");
        leftFront.set(0);
        rightFront.set(0);
        leftBack.set(0);
        rightBack.set(0);
        driveSeg2 = false;
      }
    }
    else
    {
      leftFront.setInverted(true);
      leftBack.setInverted(true);
      rightFront.setInverted(false); 
      rightBack.setInverted(false);
      Balance();
    }
  }
} 