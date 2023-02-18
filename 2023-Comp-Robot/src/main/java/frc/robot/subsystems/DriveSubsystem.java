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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.*;

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

  // Stuff for limeLight

  private boolean LowTarget = false;
  private boolean HighTarget = false;

  public ShuffleboardTab tab = Shuffleboard.getTab("Vision");

  public GenericEntry limeLightXEntry = tab.add("LimeLight tx", 0).getEntry();
  public GenericEntry limeLightYEntry = tab.add("LimeLight ty", 0).getEntry();
  public GenericEntry limeLightAEntry = tab.add("LimeLight ta", 0).getEntry();

  public boolean enableLimelight = false;
  

  public void DriveTeleop() {

    double forward = RobotContainer.XCont.getLeftY();
    double turn = RobotContainer.XCont.getRightX()*.7;
    forward = Deadzone(forward);
    turn = Deadzone(turn);
    forward = driveFilter.calculate(forward);
    turn = turnFilter.calculate(turn);
    
    if (RobotContainer.XCont.getAButtonPressed() == true){
      enableLimelight = true;
    }
    if (RobotContainer.XCont.getLeftBumperPressed() == true){
      HighTarget = false;
      LowTarget = true;
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
    }
    if (RobotContainer.XCont.getRightBumperPressed() == true){
      LowTarget = false;
      HighTarget = true;
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1 );
    } 
    if (enableLimelight == true){
      Limelight_Tracking();
    }else{
      leftFront.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
        leftBack.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
        rightFront.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
        rightBack.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);

    } 
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

    double roll = gyro.getRoll();
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
    }
    
  }

  public void DriveAuton(){
      
  }
  public void Limelight_Tracking()
  {
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

    double LimelightX = tx;
    double LimelightY = ty;
    double LimelightA = ta;
    limeLightXEntry.setDouble(LimelightX);
    limeLightYEntry.setDouble(LimelightY);
    limeLightAEntry.setDouble(LimelightA);

    double targetOffsetAngle_Vertical = ty;
    double limelightMountAngleDegrees = 19.875;
    double limelightLensHeightInches = 11;
    double goalHeightInches = 0;
    if (HighTarget == true){
      goalHeightInches = 43.9375;
    }
    else if (LowTarget == true){
      goalHeightInches = 24.00;
    }
   
    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);

    if (RobotContainer.XCont.getXButtonPressed() == true){
      enableLimelight = false;
      System.out.println("Distance to goal: "+distanceFromLimelightToGoalInches);
    }
  }
}
 