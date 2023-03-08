// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ArmSubsystem extends SubsystemBase {

    public final static CANSparkMax lowMotor  = new CANSparkMax(6, MotorType.kBrushed);
    public final static CANSparkMax upMotor  = new CANSparkMax(7, MotorType.kBrushed);

    public static Encoder upEncoder = new Encoder(4, 5, true, Encoder.EncodingType.k2X);
    public static Encoder lowEncoder = new Encoder(2, 3, false, Encoder.EncodingType.k2X);

    //SlewRateLimiter turnFilter = new SlewRateLimiter(1);

    static double startDist = 18.3125;
    static double c = 8.45;
    static double cPow = Math.pow(c, 2);
    static double b = 18.1;
    static double bPow = Math.pow(b, 2);
    static double a;
    static double lowArmAngle;
    static double lowArmRad;

    static double upEncoderRotations;
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

    boolean runIdle = false;
    boolean lowIdle = false;
    boolean upIdle = false;

    boolean runShelf = false;
    boolean lowShelf = false;
    boolean upShelf = false;

    boolean armBoolean = false;

    static double goalHeight = 0;
    static double goalDist = 0;

    Timer armTimer = new Timer();

    public void ArmInit() {
        if (armBoolean == false)
        {
            upMotor.setInverted(false);
            upEncoder.reset();

            lowMotor.setInverted(true);
            lowEncoder.reset();

            goalHeight = 48;
        }
    }
    public void ArmTeleop() {
        a = ((lowEncoder.get()/(1024*9))*.197)+startDist;
        lowArmRad = Math.acos((Math.pow(a, 2) - bPow - cPow)/(-2*b*c));
        lowArmAngle = lowArmRad*(180/Math.PI);

        upEncoderRotations = (upEncoder.get()/1024)/10;

        upArmAngle = (upEncoderRotations*((1/(7*Math.PI/360))*.197))-168;

        O1 = Math.PI*lowArmAngle/180;
        O2 = Math.PI*((180-upArmAngle)/180);

        ExtendOne = -(40.62*Math.cos(O1))+2;
        ExtendTwo = 39.25*Math.cos(O1+O2);
        if (ExtendTwo < 0)
        {
            ExtendedLength = ExtendOne - PivotToEdge;
        }
        else
        {
            ExtendedLength = ExtendOne + ExtendTwo - PivotToEdge;
        }

        Extend1 = 40.62*Math.sin(O1);
        Extend2 = 39.25*Math.sin(O1+O2);        
        ExtendedHeight = Extend1 - Extend2 + BottomToPivot;
        
        double rotateUp; 

        if (RobotContainer.XCont.getBButtonPressed())
        {
            runIdle = true;
        }
        if (RobotContainer.XCont2.getAButtonPressed())
        {
            runShelf = true;
        }
        if (Math.abs(RobotContainer.XCont2.getRightY()) > Math.abs(RobotContainer.XCont2.getRightX()))
        {
            rotateUp = RobotContainer.XCont2.getRightY();
        }
        else
        {
            rotateUp = 0;
        }
        rotateUp = Deadzone(rotateUp)*Constants.upArmSpeed;
        double rotateLow = RobotContainer.XCont2.getLeftY();
        rotateLow = Deadzone(rotateLow)*Constants.lowArmSpeed;
        if (Math.abs(rotateUp) > 0 || Math.abs(rotateLow) > 0)
        {
            Constants.runningArms = true;
            runIdle = false;
            runShelf = false;
        }
        else
        {
            Constants.runningArms = false;
        }

        if (armStop == true)
        {
            ArmStop();
        }
        else if (lowEncoder.getDistance() < -1)
        {
            lowMotor.set(-0.25);
            upMotor.set(0);
        }
        else if (upEncoder.get() < -1)
        {
            upMotor.set(-0.25);
            lowMotor.set(0);
        }
        else if (Constants.runningArms == false && runIdle == true)
        {
            IdlePosition();
        }
        else if (Constants.runningArms == false && runShelf == true)
        {
            ShelfPosition();
        }
        else
        {
            upMotor.set(rotateUp);
            lowMotor.set(rotateLow);
        }
        if (ExtendedLength >= 46)
        {
            armStop = true;
            lengthStop = true;
            heightStop = false;
        }
        else if (ExtendedHeight >= 76)
        {
            armStop = true;
            heightStop = true;
            lengthStop = false;
        }
        else
        {
            armStop = false;
            lengthStop = false;
            heightStop = false;
        }

    }

    public void ArmStop(){

        if (armStop == true && lengthStop == true)
        {
            lowMotor.set(.35);
            upMotor.set(0);
        }
        else if (armStop == true && heightStop == true)
        {
            lowMotor.set(.25);
            upMotor.set(.35);
        }
        else
        {
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

        upEncoderRotations = (upEncoder.get()/1024)/10;

        upArmAngle = (upEncoderRotations*((1/(7*Math.PI/360))*.197))-168;

        if (lowEncoder.get() > 100 && lowIdle == false)
        {
            lowMotor.set(0.8);
        }
        else
        {
            lowMotor.set(0);
            lowIdle = true;
        }
        if (upArmAngle > -150)
        {
            upMotor.set(0.75);
        }
        else if (upArmAngle > -165)
        {
            upMotor.set(0.2);
        }
        else 
        {
            upMotor.set(0);
            upIdle = true;
        }
        if (lowIdle == true && upIdle == true)
        {
            lowIdle = false;
            upIdle = false;
            runIdle = false;
        }
    }

    public void ShelfPosition(){

        a = ((lowEncoder.get()/(1024*9))*.197)+startDist;
        lowArmRad = Math.acos((Math.pow(a, 2) - bPow - cPow)/(-2*b*c));
        lowArmAngle = lowArmRad*(180/Math.PI);

        upEncoderRotations = (upEncoder.get()/1024)/10;

        upArmAngle = (upEncoderRotations*((1/(7*Math.PI/360))*.197))-168;

        if (lowArmAngle < 130)
        {
            lowMotor.set(-0.8);
            lowShelf = false;
        }
        else
        {
            lowMotor.set(0);
            lowShelf = true;
        }
        if (ExtendedHeight < 45)
        {
            System.out.println("Hello");
            upMotor.set(-0.75);
            upShelf = false;
        }
        else
        {
            System.out.println("Goodbye");
            upMotor.set(0);
            upShelf = true;
        }
        if (lowShelf == true && upShelf == true)
        {
            lowShelf = false;
            upShelf = false;
            runShelf = false;
        }
    }
    public void ScoreCube(){

        a = ((lowEncoder.get()/(1024*9))*.197)+startDist;
        lowArmRad = Math.acos((Math.pow(a, 2) - bPow - cPow)/(-2*b*c));
        lowArmAngle = lowArmRad*(180/Math.PI);

        upEncoderRotations = (upEncoder.get()/1024)/10;

        upArmAngle = (upEncoderRotations*((1/(7*Math.PI/360))*.197))-168;

        O1 = Math.PI*lowArmAngle/180;
        O2 = Math.PI*((180-upArmAngle)/180);

        ExtendOne = -(40.62*Math.cos(O1))+2;
        ExtendTwo = 39.25*Math.cos(O1+O2);
        if (ExtendTwo < 0)
        {
            ExtendedLength = ExtendOne - PivotToEdge;
        }
        else
        {
            ExtendedLength = ExtendOne + ExtendTwo - PivotToEdge;
        }

        Extend1 = 40.62*Math.sin(O1);
        Extend2 = 39.25*Math.sin(O1+O2);        
        ExtendedHeight = Extend1 - Extend2 + BottomToPivot;

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

        a = ((lowEncoder.get()/(1024*9))*.197)+startDist;
        lowArmRad = Math.acos((Math.pow(a, 2) - bPow - cPow)/(-2*b*c));
        lowArmAngle = lowArmRad*(180/Math.PI);

        upEncoderRotations = (upEncoder.get()/1024)/10;

        upArmAngle = (upEncoderRotations*((1/(7*Math.PI/360))*.197))-168;

        O1 = Math.PI*lowArmAngle/180;
        O2 = Math.PI*((180-upArmAngle)/180);

        ExtendOne = -(40.62*Math.cos(O1))+2;
        ExtendTwo = 39.25*Math.cos(O1+O2);
        if (ExtendTwo < 0)
        {
            ExtendedLength = ExtendOne - PivotToEdge;
        }
        else
        {
            ExtendedLength = ExtendOne + ExtendTwo - PivotToEdge;
        }

        Extend1 = 40.62*Math.sin(O1);
        Extend2 = 39.25*Math.sin(O1+O2);        
        ExtendedHeight = Extend1 - Extend2 + BottomToPivot;
        if (TurretSubsystem.HighTarget == true)
        {
            goalHeight = 50;
        }
        else if (TurretSubsystem.LowTarget == true)
        {
            goalHeight = 48;
        }
        //System.out.println(ExtendedLength);
        if (ExtendedHeight < goalHeight)
        {
            upMotor.set(-.25);
            lowMotor.set(0);
        }
        else if (ExtendedLength+2 < Constants.distanceFromLimelightToGoalInches)
        {
            upMotor.set(0);
            lowMotor.set(-.25);
        }
        else 
        {
            upMotor.set(0);
            lowMotor.set(0);
            ClawSubsystem.clawDrop();
        }
    }
    public void ArmAutonInit()
    {
        upMotor.setInverted(false);
        upEncoder.reset();

        lowMotor.setInverted(true);
        lowEncoder.reset();

        goalHeight = 74;
        goalDist = 45;

        armTimer.reset();
        armTimer.start();

        runIdle = true;
    }

    public void ArmAuton()
    {
        a = ((lowEncoder.get()/(1024*9))*.197)+startDist;
        lowArmRad = Math.acos((Math.pow(a, 2) - bPow - cPow)/(-2*b*c));
        lowArmAngle = lowArmRad*(180/Math.PI);

        upEncoderRotations = (upEncoder.get()/1024)/10;

        upArmAngle = (upEncoderRotations*((1/(7*Math.PI/360))*.197))-168;

        O1 = Math.PI*lowArmAngle/180;
        O2 = Math.PI*((180-upArmAngle)/180);

        ExtendOne = -(40.62*Math.cos(O1))+2;
        ExtendTwo = 39.25*Math.cos(O1+O2);
        if (ExtendTwo < 0)
        {
            ExtendedLength = ExtendOne - PivotToEdge;
        }
        else
        {
            ExtendedLength = ExtendOne + ExtendTwo - PivotToEdge;
        }

        Extend1 = 40.62*Math.sin(O1);
        Extend2 = 39.25*Math.sin(O1+O2);        
        ExtendedHeight = Extend1 - Extend2 + BottomToPivot;
        if (armTimer.get() > 1 && armTimer.get() < 4)
        {
            if (ExtendedHeight < goalHeight)
            {
                upMotor.set(-.55);
                lowMotor.set(0);
            }
            else 
            {
                upMotor.set(0);
                lowMotor.set(0);
            }
        }
        else if (armTimer.get() >= 4 && armTimer.get() < 8)
        {
            if (ExtendedLength < goalDist)
            {
                upMotor.set(0);
                lowMotor.set(-.70);
            }
            else 
            {
                upMotor.set(0);
                lowMotor.set(0);
            }
        }
        if (armTimer.get() >= 11 && armTimer.get() < 15)
        {
            if (runIdle == true)
            {
                if (lowEncoder.get() > 100 && lowIdle == false)
                {
                    lowMotor.set(0.8);
                }
                else
                {
                    lowMotor.set(0);
                    lowIdle = true;
                }
                if (upArmAngle > -150)
                {
                    upMotor.set(0.75);
                }
                else if (upArmAngle > -165)
                {
                    upMotor.set(0.2);
                }
                else 
                {
                    upMotor.set(0);
                    upIdle = true;
                }
                if (lowIdle == true && upIdle == true)
                {
                    lowIdle = false;
                    upIdle = false;
                    runIdle = false;
                }
            }
        }
    }
}