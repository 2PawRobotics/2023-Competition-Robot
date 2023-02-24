// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ClawSubsystem extends SubsystemBase {

    /*private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
    private final ColorMatch colorMatcher = new ColorMatch();

    private final Color yellow = new Color(1, 1, 1);
    private final Color purple = new Color(0.5, 0, 0.5);

    private final CANSparkMax clawMotor = new CANSparkMax(8, MotorType.kBrushed);

    public static Encoder clawEncoder = new Encoder(5, 6, false, Encoder.EncodingType.k2X);

    String colorStringCheck;

    Boolean clawOpen = true;*/

    public void ClawInit(){

        /*colorMatcher.addColorMatch(yellow);  
        colorMatcher.addColorMatch(purple);

        clawEncoder.setDistancePerPulse(Constants.encPulse);
        clawEncoder.reset();*/

    }

    public void ClawTeleop(){

        /*Color detectedColor = colorSensor.getColor();

        String colorString;
        ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

        if (match.color == purple) 
        {
            colorString = "Purple";
        } 
        else if (match.color == yellow)
        {
            colorString = "Yellow";
        } 
        else 
        {
            colorString = "Unknown";
        }
        if (colorString != colorStringCheck){
            colorStringCheck = colorString;
            System.out.println("Seeing: "+colorString);
        }*/

    }

    public void ClawInteract(){

        /*if (clawOpen == true && clawEncoder.get() < 2048) 
        {
            clawMotor.set(-.01);
        }
        else if (clawOpen == false && clawEncoder.get() > 1)
        {
            clawMotor.set(.01);
        }
        else
        {
            clawMotor.set(0);
        }*/

    }
}