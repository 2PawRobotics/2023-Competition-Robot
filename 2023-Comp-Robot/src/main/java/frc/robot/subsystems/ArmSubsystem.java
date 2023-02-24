// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ArmSubsystem extends SubsystemBase {

    public final static CANSparkMax lowMotor  = new CANSparkMax(6, MotorType.kBrushed);
    public final static CANSparkMax upMotor  = new CANSparkMax(7, MotorType.kBrushed);

    public static Encoder upEncoder = new Encoder(4, 5, false, Encoder.EncodingType.k2X);
    public static Encoder lowEncoder = new Encoder(2, 3, false, Encoder.EncodingType.k2X);

    SlewRateLimiter turnFilter = new SlewRateLimiter(2);

    static double lowDistPerTic = 1/((1024*100)/360);
    static double upDistPerTic = 1/((1024*100)/360);

    boolean armStop;
    boolean lengthStop;
    boolean heightStop;
    double lowMotorStop = 59.5;

    static double O1;
    static double O2;

    static double ExtendOne;
    static double ExtendTwo;
    static double PivotToEdge = 14;
    static double ExtendedLength;

    static double Extend1;
    static double Extend2;
    static double BottomToPivot = 8.95;
    static double ExtendedHeight;

    public void ArmInit() {

        lowMotor.setInverted(true);
		
        lowEncoder.setDistancePerPulse(Constants.encPulse);
        upEncoder.setDistancePerPulse(Constants.encPulse);

    }
    public void ArmTeleop() {

        O1 = lowEncoder.getDistance()*lowDistPerTic;
        O2 = upEncoder.getDistance()*upDistPerTic;

        ExtendOne = 40.62*Math.cos(O1);
        ExtendTwo = 39.25*Math.cos(O1+O2);
        ExtendedLength = ExtendOne + ExtendTwo - PivotToEdge;

        Extend1 = 40.62*Math.sin(O1);
        Extend2 = 39.25*Math.sin(O2);
        ExtendedHeight = Extend1 + Extend2 + BottomToPivot;

        double rotateLow = RobotContainer.XCont2.getLeftY();
        rotateLow = Deadzone(rotateLow);
        rotateLow = turnFilter.calculate(rotateLow);
        double rotateUp = RobotContainer.XCont2.getRightY();
        rotateUp = Deadzone(rotateUp);
        rotateUp = turnFilter.calculate(rotateUp);

        if (armStop == true)
        {
            ArmStop();
        }
        else
        {
            upMotor.set(rotateLow);
            lowMotor.set(rotateUp);
        }
        if (ExtendedLength >= 47.5)
        {
            armStop = true;
            lengthStop = true;
        }
        else if (ExtendedHeight >= 59.5)
        {
            armStop = true;
            heightStop = true;
        }
        else if (O1 >= lowMotorStop)
        {
            lowMotor.set(-0.1);
        }
        else if (O1 <= -lowMotorStop)
        {
            lowMotor.set(0.1);
        }
        else
        {
            armStop = false;
            lengthStop = false;
            heightStop = false;
        }

    }

    public void ArmStop(){

        lowMotor.set(0);
        if (armStop == true && lengthStop == true)
        {
            lowMotor.set(-.01);
        }
        else if (armStop == true && heightStop == true)
        {
            lowMotor.set(-.01);
            upMotor.set(.01);
        }
        else
        {
            lowMotor.set(0);
            armStop = false;
            lengthStop = false;
            heightStop = false;
        }
    }

    public double Deadzone(double value){
        /* Upper deadzone */
        if (value >= +0.05){
            value = value*Constants.armSpeed;
            return value;
        }
      /* Lower deadzone */
        else if (value <= -0.05){
            value = value*Constants.armSpeed;
            return value;
        }
      /* Outside deadzone */
        else{return 0;}
      }

    public void IdlePosition(){

        if (O1>-1.5)
        {
            lowMotor.set(-.01);
        }
        else if (O1<-2.5)
        {
            lowMotor.set(.01);
        }
        else
        {
            lowMotor.set(0);
        }
        if (O2>178.5)
        {
            upMotor.set(-.01);
        }
        else if (O2<177.5)
        {
            upMotor.set(.01);
        }
        else
        {
            upMotor.set(0);
        }

    }

    public void ShelfPosition(){

        if (O1>10.5)
        {
            lowMotor.set(-.01);
        }
        else if (O1<9.5)
        {
            lowMotor.set(.01);
        }
        else
        {
            lowMotor.set(0);
        }
        if (O2>90.5)
        {
            upMotor.set(-.01);
        }
        else if (O2<89.5)
        {
            upMotor.set(.01);
        }
        else
        {
            upMotor.set(0);
        }

    }
    public void ScoreCube(){

        O1 = lowEncoder.getDistance()*lowDistPerTic;
        O2 = upEncoder.getDistance()*upDistPerTic;
        Extend1 = 40.62*Math.sin(O1);
        Extend2 = 39.25*Math.sin(O2);
        ExtendedHeight = Extend1 + Extend2 + BottomToPivot;

        double goalHeight = 0;

        if (TurretSubsystem.HighTarget == true)
        {
            goalHeight = 48;
        }
        else if (TurretSubsystem.LowTarget == true)
        {
            goalHeight = 36;
        }

        if (ExtendedHeight < goalHeight)
        {
            upMotor.set(.1);
            lowMotor.set(0);
        }
    }

    public static void ScoreCone(){

        O1 = lowEncoder.getDistance()*lowDistPerTic;
        O2 = upEncoder.getDistance()*upDistPerTic;

        ExtendOne = 40.62*Math.cos(O1);
        ExtendTwo = 39.25*Math.cos(O1+O2);
        ExtendedLength = ExtendOne + ExtendTwo - PivotToEdge;

        Extend1 = 40.62*Math.sin(O1);
        Extend2 = 39.25*Math.sin(O2);
        ExtendedHeight = Extend1 + Extend2 + BottomToPivot;
        double goalHeight = 0;
        if (TurretSubsystem.HighTarget == true)
        {
            goalHeight = 50;
        }
        else if (TurretSubsystem.LowTarget == true)
        {
            goalHeight = 36;
        }

        if (ExtendedHeight < goalHeight)
        {
            upMotor.set(.1);
            lowMotor.set(0);
        }
        else if (ExtendedLength < Constants.distanceFromLimelightToGoalInches)
        {
            upMotor.set(0);
            lowMotor.set(.1);
        }
        else 
        {
            Constants.havePlacedCone = true;
            upMotor.set(0);
            lowMotor.set(0);
        }
    }
}