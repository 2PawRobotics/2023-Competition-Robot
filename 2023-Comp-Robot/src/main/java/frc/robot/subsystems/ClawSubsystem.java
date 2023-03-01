// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Encoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ClawSubsystem extends SubsystemBase {


    private final static CANSparkMax clawMotor = new CANSparkMax(8, MotorType.kBrushed);

    public static Encoder clawEncoder = new Encoder(6, 7, false, Encoder.EncodingType.k2X);

    //String colorStringCheck;

    boolean clawOpen = true;
    boolean clawRun = false;
    boolean clawBoolean = false;
    boolean clawStart = false;
    

    public void ClawInit(){

        if (clawBoolean == false){
            clawEncoder.reset();
            clawMotor.set(0);
            clawBoolean = true;
        }
        clawMotor.set(0);
    }

    public void ClawTeleop(){
        
        if (RobotContainer.XCont2.getXButtonPressed())
        {
            clawRun = true;
        }
        if (clawRun == true)
        {
            ClawInteract();
        }
    }

    public void ClawInteract(){
        
        double clawDist = .0000766895*clawEncoder.get();
        if (clawRun == true)
        {
            if (clawOpen == true) 
            {
                if (clawDist < 5.75)
                {
                    clawMotor.set(.3);
                }
                else
                {
                    clawMotor.set(0);
                    clawOpen = false;
                    clawRun = false;
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
                    clawRun = false;
                }
            }
            else
            {
                clawMotor.set(0);
                clawRun = false;
            }
        }
    }
}