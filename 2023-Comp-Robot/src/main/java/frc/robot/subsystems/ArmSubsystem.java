// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ArmSubsystem extends SubsystemBase {

    public final static CANSparkMax lowMotor  = new CANSparkMax(6, MotorType.kBrushed);
    public final static CANSparkMax upMotor  = new CANSparkMax(7, MotorType.kBrushed);

    public static Encoder upEncoder = new Encoder(4, 5, false, Encoder.EncodingType.k2X);
    public static Encoder lowEncoder = new Encoder(2, 3, false, Encoder.EncodingType.k2X);

    //SlewRateLimiter turnFilter = new SlewRateLimiter(1);

    static double startDist = 17.75;
    static double c = Math.pow(8.45, 2);
    static double b = Math.pow(18.1, 2);
    static double a = 0;
    static double lowArmAngle;

    static double upEncoderRotations = 0;
    static double upArmAngle;

    boolean armStop;
    boolean lengthStop;
    boolean heightStop;
    double lowMotorStop = 60;

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

    boolean armBoolean = false;

    public void ArmInit() {
        if (armBoolean == false)
        {
            upMotor.setInverted(true);
            upEncoder.setDistancePerPulse(a);
            upEncoder.reset();

            lowMotor.setInverted(true);
            lowEncoder.setDistancePerPulse(a);
            lowEncoder.reset();
        }

    }
    public void ArmTeleop() {

        a = lowEncoder.getDistance()*(1/((1024*10)*.2));
        lowArmAngle = Math.acos((b + c - Math.pow((a+startDist), 2))/(2*18.1*8.45));

        upEncoderRotations = (upEncoder.getDistance()/1024)/10;

        upArmAngle = upEncoderRotations*.4363278;

        O1 = 180 - lowArmAngle;
        O2 = upArmAngle;

        ExtendOne = 40.62*Math.cos(O1);
        ExtendTwo = 39.25*Math.cos(O1+O2);
        ExtendedLength = ExtendOne + ExtendTwo - PivotToEdge;

        Extend1 = 40.62*Math.sin(O1);
        Extend2 = 39.25*Math.sin(O2);
        ExtendedHeight = Extend1 + Extend2 + BottomToPivot;

        double rotateLow = RobotContainer.XCont2.getLeftY();
        rotateLow = Deadzone(rotateLow)*Constants.lowArmSpeed;
        double rotateUp = RobotContainer.XCont2.getRightY();
        rotateUp = Deadzone(rotateUp)*Constants.upArmSpeed;

        if (armStop == true)
        {
            System.out.println("Hello");
            ArmStop();
        }
        else if (lowEncoder.get() <= 0)
        {
            System.out.println("Hello");
            lowMotor.set(-0.25);
        }
        else
        {
            upMotor.set(rotateUp);
            lowMotor.set(rotateLow);
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
        if (value >= +0.1){
            return value;
        }
      /* Lower deadzone */
        else if (value <= -0.1){
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

        a = lowEncoder.getDistance()*(1/((1024*100)*.2));

        lowArmAngle = Math.acos((b + c - Math.pow((a+startDist), 2))/(2*18.1*8.45));

        upEncoderRotations = (upEncoder.getDistance()/1024)/100;

        upArmAngle = upEncoderRotations*.4363278;

        O1 = 180 - lowArmAngle;
        O2 = upArmAngle;

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
        }
        else
        {
            upMotor.set(0);
        }
    }

    public static void ScoreCone(){

        a = lowEncoder.getDistance()*(1/((1024*100)*.2));

        lowArmAngle = Math.acos((b + c - Math.pow((a+startDist), 2))/(2*18.1*8.45));

        upEncoderRotations = (upEncoder.getDistance()/1024)/100;

        upArmAngle = upEncoderRotations*.4363278;

        O1 = 180 - lowArmAngle;
        O2 = upArmAngle;

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