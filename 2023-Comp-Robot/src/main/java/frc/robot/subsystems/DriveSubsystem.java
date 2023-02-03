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
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.*;

import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveSubsystem extends SubsystemBase {

  // Drive Motors
  public static WPI_TalonFX leftFront = new WPI_TalonFX(1, "rio"); 
  public static WPI_TalonFX leftBack = new WPI_TalonFX(2, "rio");
  public static WPI_TalonFX rightFront = new WPI_TalonFX(3, "rio");
  public static WPI_TalonFX rightBack = new WPI_TalonFX(4, "rio");

  // Gyro
  AHRS gyro = new AHRS(SerialPort.Port.kMXP);

  // Motor Values 
	final TalonFXInvertType kInvertType = TalonFXInvertType.Clockwise; 
	final NeutralMode kBrakeDurNeutral = NeutralMode.Coast;
  SlewRateLimiter driveFilter = new SlewRateLimiter(0.4);
  SlewRateLimiter turnFilter = new SlewRateLimiter(0.25);

  // Drive Encoder Units
  final double kUnitsPerRevolution = (Math.PI*6)/8.45; 

  // Stuff for limeLight
  private boolean limelightHasValidTarget = false;
  private double limelightDriveCommand = 0.0;
  private double limelightSteerCommand = 0.0;

  private double hLowTarget = 24.00;
  private double hHighTarget = 43.81;

  public ShuffleboardTab tab = Shuffleboard.getTab("Vision");
  public GenericEntry limeLightXEntry = tab.add("LimeLight tx", 0).getEntry();
  public GenericEntry limeLightYEntry = tab.add("LimeLight ty", 0).getEntry();
  public GenericEntry limeLightAEntry = tab.add("LimeLight ta", 0).getEntry();
  

  public void DriveTeleop() {

    Update_Limelight_Tracking();

    double forward = RobotContainer.XCont.getLeftY();
    double turn = RobotContainer.XCont.getRightX()*.75;
    forward = Deadzone(forward);
    turn = Deadzone(turn);
    forward = driveFilter.calculate(forward);
    turn = turnFilter.calculate(turn);

    boolean auto = RobotContainer.XCont.getAButton();

    if (auto == true){
      if (limelightHasValidTarget){
        leftFront.set(ControlMode.PercentOutput, limelightDriveCommand, DemandType.ArbitraryFeedForward, -limelightSteerCommand);
        leftBack.set(ControlMode.PercentOutput, limelightDriveCommand, DemandType.ArbitraryFeedForward, -limelightSteerCommand);
        rightFront.set(ControlMode.PercentOutput, limelightDriveCommand, DemandType.ArbitraryFeedForward, +limelightSteerCommand);
        rightBack.set(ControlMode.PercentOutput, limelightDriveCommand, DemandType.ArbitraryFeedForward, +limelightSteerCommand);
      }else{
        leftFront.set(0);
        leftBack.set(0);
        rightFront.set(0);
        rightBack.set(0);}
    }else{
      leftFront.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
      leftBack.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
      rightFront.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
      rightBack.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
    }

    double appliedMotorOutputLeft = leftFront.getMotorOutputPercent();
		double selSenPosLeft = (leftFront.getSelectedSensorPosition(0)/2048)*((Math.PI*6)/8.45); /* position units */
		double selSenVelLeft = leftFront.getSelectedSensorVelocity(0); /* position units per 100ms */

		double pos_RotationsLeft = (double) selSenPosLeft / kUnitsPerRevolution;
		double vel_RotPerSecLeft = (double) selSenVelLeft / kUnitsPerRevolution * 10; /* scale per100ms to perSecond */
		double vel_RotPerMinLeft = vel_RotPerSecLeft * 60.0;

    double appliedMotorOutputRight = rightFront.getMotorOutputPercent();
		double selSenPosRight = (rightFront.getSelectedSensorPosition(0)/2048)*((Math.PI*6)/8.45); /* position units */
		double selSenVelRight = rightFront.getSelectedSensorVelocity(0); /* position units per 100ms */

		double pos_RotationsRight = (double) selSenPosRight / kUnitsPerRevolution;
		double vel_RotPerSecRight = (double) selSenVelRight / kUnitsPerRevolution * 10; /* scale per100ms to perSecond */
		double vel_RotPerMinRight = vel_RotPerSecRight * 60.0;

    if (++Constants.printLoops >= 10) {
			Constants.printLoops = 0;
      System.out.println("Left Motor:");
      System.out.printf("Motor speed : %.2f | ", appliedMotorOutputLeft);
			System.out.printf("Position (Inches) : %.2f | ", selSenPosLeft);
			System.out.printf("Velocity (unitsPer100m) : %.2f | ", selSenVelLeft);
			System.out.printf("Rotations : %.3f | ", pos_RotationsLeft);
			System.out.printf("RPS : %.1f | ", vel_RotPerSecLeft);
			System.out.printf("RPM : %.1f | ", vel_RotPerMinLeft);
			System.out.println("Right Motor:");
      System.out.printf("Motor speed : %.2f | ", appliedMotorOutputRight);
			System.out.printf("Position (Inches) : %.2f | ", selSenPosRight);
			System.out.printf("Velocity (unitsPer100m) : %.2f | ", selSenVelRight);
			System.out.printf("Rotations : %.3f | ", pos_RotationsRight);
			System.out.printf("RPS : %.1f | ", vel_RotPerSecRight);
			System.out.printf("RPM : %.1f | ", vel_RotPerMinRight);
			System.out.println("");
    }
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

    double pitchAngle = gyro.getPitch();

    if (pitchAngle > 5){
      leftFront.set(ControlMode.PercentOutput, .1);
      leftBack.set(ControlMode.PercentOutput, .1);
      rightFront.set(ControlMode.PercentOutput, .1);
      rightBack.set(ControlMode.PercentOutput, .1);
    }else if (pitchAngle < -5){
      leftFront.set(ControlMode.PercentOutput, -.1);
      leftBack.set(ControlMode.PercentOutput, -.1);
      rightFront.set(ControlMode.PercentOutput, -.1);
      rightBack.set(ControlMode.PercentOutput, -.1);
    }else{
      leftFront.set(ControlMode.PercentOutput, 0);
      leftBack.set(ControlMode.PercentOutput, 0);
      rightFront.set(ControlMode.PercentOutput, 0);
      rightBack.set(ControlMode.PercentOutput, 0);
    }
  }

  public void DriveAuton(){

  /*double forward = RobotContainer.XCont.getLeftY();
    double turn = RobotContainer.XCont.getRightX();
    forward = Deadzone(forward);
    turn = Deadzone(turn);

    leftFront.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);

    double appliedMotorOutput = leftFront.getMotorOutputPercent();
		double selSenPos = (leftFront.getSelectedSensorPosition(0)/2048)*(Math.PI*6); /* position units */
		//double selSenVel = leftFront.getSelectedSensorVelocity(0); /* position units per 100ms */

		/* scaling depending on what user wants */
		/*double pos_Rotations = (double) selSenPos / kUnitsPerRevolution;
		double vel_RotPerSec = (double) selSenVel / kUnitsPerRevolution * 10; /* scale per100ms to perSecond */
		/*double vel_RotPerMin = vel_RotPerSec * 60.0;

    if (++Constants.printLoops >= 10) {
			Constants.printLoops = 0;
      System.out.printf("Motor speed : %.2f | ", appliedMotorOutput);
			System.out.printf("Position (Inches) : %.2f | ", selSenPos);
			System.out.printf("Velocity (unitsPer100m) : %.2f | ", selSenVel);
			System.out.printf("Rotations : %.3f | ", pos_Rotations);
			System.out.printf("RPS : %.1f | ", vel_RotPerSec);
			System.out.printf("RPM : %.1f | ", vel_RotPerMin);
			System.out.println("");
		}*/
      
  }
  public void Update_Limelight_Tracking()
  {
        // These numbers must be tuned for your Robot!  Be careful!
        final double STEER_K = 0.03;                    // how hard to turn toward the target
        final double DRIVE_K = 0.26;                    // how hard to drive fwd toward the target
        final double DESIRED_TARGET_AREA = 13.0;        // Area of the target when the robot reaches the wall
        final double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast

        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

        if (tv < 1.0)
        {
          limelightHasValidTarget = false;
          limelightDriveCommand = 0.0;
          limelightSteerCommand = 0.0;
          return;
        }

        limelightHasValidTarget = true;

        //post to smart dashboard periodically
        double LimelightX = tx;
        double LimelightY = tx;
        double LimelightA = tx;
        limeLightXEntry.setDouble(LimelightX);
        limeLightYEntry.setDouble(LimelightY);
        limeLightAEntry.setDouble(LimelightA);

        // Start with proportional steering
        double steer_cmd = tx * STEER_K;
        limelightSteerCommand = steer_cmd;

        // try to drive forward until the target area reaches our desired area
        double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

        // don't let the robot drive too fast into the goal
        if (drive_cmd > MAX_DRIVE)
        {
          drive_cmd = MAX_DRIVE;
        }
        limelightDriveCommand = drive_cmd;
  }
}
 