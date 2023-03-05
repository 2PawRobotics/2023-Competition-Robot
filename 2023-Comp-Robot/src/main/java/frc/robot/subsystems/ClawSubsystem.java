// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Encoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ClawSubsystem extends SubsystemBase {


    private final static CANSparkMax clawMotor = new CANSparkMax(8, MotorType.kBrushed);

    public static Encoder clawEncoder = new Encoder(6, 7, false, Encoder.EncodingType.k2X);

    //String colorStringCheck;

    static boolean clawOpen = true;
    boolean clawRunCone = false;
    boolean clawRunCube = false;
    boolean clawBoolean = false;
    boolean clawStart = false;
    static boolean clawHasDropped = false;

    static double clawDist;
    

    public void ClawInit(){

        if (clawBoolean == false){
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
                if (clawDist < 6.1)
                {
                    if (clawMotor.getOutputCurrent() > 20)
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
                else if (clawDist > .75)
                {
                    clawMotor.set(-.125);
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
                if (clawDist < 2.5)
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
                if (clawDist > 4)
                {
                    clawMotor.set(-.4);
                }
                else if (clawDist > .75)
                {
                    clawMotor.set(-.125);
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
    public static void clawDrop(){

        if (clawHasDropped == false)
        {
            if (clawDist > 4)
            {
                clawMotor.set(-.4);
            }
            else if (clawDist > .75)
            {
                clawMotor.set(-.125);
            }
            else
            {
                clawMotor.set(0);
                clawOpen = true;
                clawHasDropped = true;
                Constants.havePlacedCone = true;
            }
        }
    }
}