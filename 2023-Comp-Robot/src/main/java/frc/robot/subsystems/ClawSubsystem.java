// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ClawSubsystem extends SubsystemBase {

    private final static CANSparkMax clawMotor = new CANSparkMax(8, MotorType.kBrushed);

    public static Encoder clawEncoder = new Encoder(6, 7, false, Encoder.EncodingType.k2X);

    boolean clawOpen = true;
    boolean clawRunCone = false;
    boolean clawRunCube = false;
    boolean clawBoolean = false;

    double clawDist;

    /* Auton */
    Timer clawTimer = new Timer();

    public void ClawInit()
    {
        if (clawBoolean == false)
        {
            clawEncoder.reset();
            clawMotor.set(0);
            clawBoolean = true;
        }
        clawMotor.set(0);
    }

    public void ClawTeleop(){
        
        if (RobotContainer.XCont2.getYButtonPressed())
        {
            clawRunCube = true;
        }
        else if (RobotContainer.XCont2.getXButtonPressed())
        {
            clawRunCone = true;
        }
        if (clawRunCube == true)
        {
            ClawInteractCube();
        }
        else if (clawRunCone == true)
        {
            ClawInteractCone();
        }
    }

    public void ClawInteractCone(){
        
        clawDist = .0000977202576*clawEncoder.get();
        if (clawRunCone == true)
        {
            if (clawOpen == true) 
            {
                if (clawDist < 6.6)
                {
                    System.out.println(clawMotor.getOutputCurrent());
                    if (clawMotor.getOutputCurrent() < 35)
                    {
                        clawMotor.set(.4);
                    }
                    else
                    {
                        clawMotor.set(0);
                        clawOpen = false;
                        clawRunCone = false;
                    }
                }
                else
                {
                    clawMotor.set(0);
                    clawOpen = false;
                    clawRunCone = false;
                }
            }
            else if (clawOpen == false)
            {
                if (clawDist > 4)
                {
                    clawMotor.set(-.25);
                }
                else if (clawDist > .3)
                {
                    clawMotor.set(-.25);
                }
                else
                {
                    clawMotor.set(0);
                    clawOpen = true;
                    clawRunCone = false;
                }
            }
            else
            {
                clawMotor.set(0);
                clawRunCone = false;
            }
        }
    }
    public void ClawInteractCube(){
        
        clawDist = .0000977202576*clawEncoder.get();
        if (clawRunCube == true)
        {
            if (clawOpen == true) 
            {
                if (clawDist < 2.6)
                {
                    System.out.println(clawDist);
                    clawMotor.set(.5);
                }
                else
                {
                    clawMotor.set(0);
                    clawOpen = false;
                    clawRunCube = false;
                }
            }
            else if (clawOpen == false)
            {
                if (clawDist > 2)
                {
                    clawMotor.set(-.4);
                }
                else if (clawDist > .3)
                {
                    clawMotor.set(-.25);
                }
                else
                {
                    clawMotor.set(0);
                    clawOpen = true;
                    clawRunCube = false;
                }
            }
            else
            {
                clawMotor.set(0);
                clawRunCube = false;
            }
        }
    }

    public void ClawAutonInit()
    {
        clawTimer.reset();
        clawTimer.start();
        clawEncoder.reset();
        clawMotor.set(0);
    }

    public void ClawAuton()
    {
        if (clawOpen == true && clawTimer.get() <= 2)
        {
            clawRunCube = true;
            ClawInteractCube();
        }
        else if (clawOpen == false && clawTimer.get() >= 5)
        {
            clawRunCube = true;
            ClawInteractCube();
        }
    }
}