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

    static double startDist = 18.5;
    static double c = 11.75;
    static double cPow = Math.pow(c, 2);
    static double b = 18.13;
    static double bPow = Math.pow(b, 2);
    static double a;
    static double lowArmAngle;
    static double lowArmRad;

    static double upEncoderRotations;
    static double upArmAngle;

    boolean armStop;
    boolean lengthStop;
    boolean heightStop;
    boolean groundStop;

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

    boolean runPickup = false;
    boolean lowPickup = false;
    boolean upPickup = false;

    boolean armBoolean = false;

    double goalHeight = 0;
    double goalDist = 0;
    boolean runGoal = false;
    boolean lowGoal = false;
    boolean highGoal = false;

    Timer armTimer = new Timer();

    public void ArmInit() 
    {
        if (armBoolean == false)
        {
            upMotor.setInverted(false);
            upEncoder.reset();

            lowMotor.setInverted(true);
            lowEncoder.reset();

            goalHeight = 45;
            goalDist = 30;
        }
    }
    public void ArmTeleop() 
    {
        a = ((lowEncoder.get()/(1024*9))*.197)+startDist;
        lowArmRad = Math.acos((Math.pow(a, 2) - bPow - cPow)/(-2*b*c));
        lowArmAngle = lowArmRad*(180/Math.PI);
        upEncoderRotations = (upEncoder.get()/1024)/9;

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

        int dPadValue = RobotContainer.XCont2.getPOV();
        if (dPadValue == 0)
        {
            goalHeight = 75;
            goalDist = 35;
        }
        else if (dPadValue == 180)
        {
            goalHeight = 56.5;
            goalDist = 29;
        }

        if (RobotContainer.XCont.getBButtonPressed())
        {
            runIdle = true;
        }
        if (RobotContainer.XCont2.getAButtonPressed())
        {
            runShelf = true;
        }
        if (RobotContainer.XCont2.getBButtonPressed())
        {
            runPickup = true;
        }
        if (RobotContainer.XCont2.getRightTriggerAxis() == 1)
        {
            runGoal = true;
        }

        double rotateUp = RobotContainer.XCont2.getRightY();
        rotateUp = Deadzone(rotateUp)*Constants.upArmSpeed;
        
        double rotateLow = RobotContainer.XCont2.getLeftY();
        rotateLow = Deadzone(rotateLow)*Constants.lowArmSpeed;

        if (Math.abs(rotateUp) > 0 || Math.abs(rotateLow) > 0)
        {
            Constants.runningArms = true;
            runIdle = false;
            runShelf = false;
            runGoal = false;
            runPickup = false;
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
        else if (Constants.runningArms == false && runGoal == true)
        {
            GoalPosition();
        }
        else if (Constants.runningArms == false && runIdle == true)
        {
            IdlePosition();
        }
        else if (Constants.runningArms == false && runPickup == true)
        {
            PickupPosition();
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
            groundStop = false;
        }
        else if (ExtendedHeight >= 76)
        {
            armStop = true;
            heightStop = true;
            lengthStop = false;
            groundStop = false;
        }
        else if (ExtendedHeight <= 3)
        {
            armStop = true;
            heightStop = false;
            lengthStop = false;
            groundStop = true;
        }
        else
        {
            armStop = false;
            lengthStop = false;
            heightStop = false;
            groundStop = false;
        }

    }

    public void ArmStop()
    {
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
        else if (armStop == true && groundStop == true)
        {
            lowMotor.set(.35);
            upMotor.set(0);
        }
        else
        {
            armStop = false;
            lengthStop = false;
            heightStop = false;
        }
    }

    public double Deadzone(double value)
    {
        if (value >= +0.1)
        {
            return value;
        }
        else if (value <= -0.1)
        {
            return value;
        }
        else
        {
            return 0;
        }
    }

    public void GoalPosition()
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
        if (ExtendedHeight < goalHeight && highGoal == false)
        {
            upMotor.set(-.85);

        }
        else 
        {
            upMotor.set(0);
            highGoal = true;
        }
        if (ExtendedLength < goalDist && lowGoal == false)
        {
            if (ExtendedHeight > goalHeight-5)
            {
                lowMotor.set(-1);
            }
            else
            {
                lowMotor.set(-.25);
            }
        }
        else 
        {
            lowMotor.set(0);
            lowGoal = true;
        }
        if (lowGoal == true && highGoal == true)
        {
            lowGoal = false;
            highGoal = false;
            runGoal = false;
        }
}

    public void IdlePosition()
    {
        upEncoderRotations = (upEncoder.get()/1024)/10;
        upArmAngle = (upEncoderRotations*((1/(7*Math.PI/360))*.197))-168;

        if (lowEncoder.get() > 95 && lowIdle == false)
        {
            lowMotor.set(1);
        }
        else
        {
            lowMotor.set(0);
            lowIdle = true;
        }
        if (upArmAngle > -135)
        {
            upMotor.set(1);
        }
        else if (upArmAngle > -145)
        {
            upMotor.set(0.5);
        }
        else if (upArmAngle > -155)
        {
            upMotor.set(0.1);
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

    public void PickupPosition()
    {
        a = ((lowEncoder.get()/(1024*9))*.197)+startDist;
        lowArmRad = Math.acos((Math.pow(a, 2) - bPow - cPow)/(-2*b*c));
        lowArmAngle = lowArmRad*(180/Math.PI);

        upEncoderRotations = (upEncoder.get()/1024)/10;

        upArmAngle = (upEncoderRotations*((1/(7*Math.PI/360))*.197))-168;

        if (lowArmAngle < 113.5 && lowPickup == false)
        {
            lowMotor.set(-0.8);
            lowPickup = false;
        }
        else if (lowArmAngle > 118.5 && lowPickup == false)
        {
            lowMotor.set(0.8);
            lowPickup = false;
        }
        else
        {
            lowMotor.set(0);
            lowPickup = true;
        }
        if (upArmAngle < -145 && upPickup == false)
        {
            upMotor.set(-0.75);
            upPickup = false;
        }
        else if (upArmAngle > -140 && upPickup == false)
        {
            upMotor.set(0.75);
            upPickup = false;
        }
        else
        {
            upMotor.set(0);
            upPickup = true;
        }
        if (lowPickup == true && upPickup == true)
        {
            lowPickup = false;
            upPickup = false;
            runPickup = false;
        }
    }

    public void ShelfPosition()
    {
        a = ((lowEncoder.get()/(1024*9))*.197)+startDist;
        lowArmRad = Math.acos((Math.pow(a, 2) - bPow - cPow)/(-2*b*c));
        lowArmAngle = lowArmRad*(180/Math.PI);

        upEncoderRotations = (upEncoder.get()/1024)/10;

        upArmAngle = (upEncoderRotations*((1/(7*Math.PI/360))*.197))-168;

        if (lowArmAngle < 90)
        {
            lowMotor.set(-0.8);
            lowShelf = false;
        }
        else
        {
            lowMotor.set(0);
            lowShelf = true;
        }
        if (upArmAngle < -80)
        {
            upMotor.set(-0.75);
            upShelf = false;
        }
        else
        {
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

    public void ArmAutonInit()
    {
        upMotor.setInverted(false);
        upEncoder.reset();

        lowMotor.setInverted(true);
        lowEncoder.reset();

        goalHeight = 74;
        goalDist = 40;

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
        if (armTimer.get() > 1 && armTimer.get() < 3)
        {
            if (ExtendedHeight < goalHeight)
            {
                upMotor.set(-.65);
                lowMotor.set(0);
            }
            else 
            {
                upMotor.set(0);
                lowMotor.set(0);
            }
        }
        else if (armTimer.get() >= 3 && armTimer.get() < 4.5)
        {
            if (ExtendedLength < goalDist)
            {
                upMotor.set(0);
                lowMotor.set(-.8);
            }
            else 
            {
                upMotor.set(0);
                lowMotor.set(0);
            }
        }
        if (armTimer.get() >= 5.5 && armTimer.get() < 15)
        {
            if (runIdle == true)
            {
                if (lowEncoder.get() > 100 && lowIdle == false)
                {
                    lowMotor.set(1);
                }
                else
                {
                    lowMotor.set(0);
                    lowIdle = true;
                }
                if (armTimer.get() >= 6.5 && armTimer.get() < 15)
                {
                    if (upArmAngle > -145)
                    {
                        upMotor.set(1);
                    }
                    else if (upArmAngle > -155)
                    {
                        upMotor.set(0.5);
                    }
                    else if (upArmAngle > -165)
                    {
                        upMotor.set(0.1);
                    }
                    else 
                    {
                        upMotor.set(0);
                        upIdle = true;
                    }
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