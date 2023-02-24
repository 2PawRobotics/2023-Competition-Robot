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
  final double kUnitsPerRevolution = (Math.PI*6)/8.45; 
  

  public void DriveTeleop() {

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
    //System.out.println("Gyro angle: "+gyro.getYaw());

  }

  public void DriveInitialize(){

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
  
  public double Deadzone(double value){
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

  public void DriveAutonInit(){

    TalonFXConfiguration configs = new TalonFXConfiguration();
			
		configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
			
		leftFront.configAllSettings(configs);

    leftFront.setSelectedSensorPosition(0);

  }

  public void Balance(){

    /*double roll = gyro.getRoll();
    if (roll > 5){
      leftFront.set(-.1);
      leftBack.set(-.1);
      rightFront.set(-.1);
      rightBack.set(-.1);
    }else if (roll < -5){
      leftFront.set(.1);
      leftBack.set(.1);
      rightFront.set(.1);
      rightBack.set(.1);
    }else{
      leftFront.set(0);
      leftBack.set(0);
      rightFront.set(0);
      rightBack.set(0);
    }*/
    
  }

  public void DriveAuton(){
      
  }
  
} 