// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ArmSubsystem extends SubsystemBase {

    private static WPI_TalonFX lowMotor = new WPI_TalonFX(6, "rio");
    private final CANSparkMax upMotor  = new CANSparkMax(5, MotorType.kBrushed);

    public static Encoder upEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k2X);

    SlewRateLimiter turnFilter = new SlewRateLimiter(2);

    double lowDistPerTic = 1/((2048*225)/360);
    double upDistPerTic = 1/((1024*100)/360);

    boolean armStop;
    boolean lengthStop;
    boolean heightStop;
    double lowMotorStop = 59.5;

    double O1;
    double O2;

    double ExtendOne;
    double ExtendTwo;
    double PivotToEdge = 14;
    double ExtendedLength;

    double Extend1;
    double Extend2;
    double BottomToPivot = 8.95;
    double ExtendedHeight;

    public void ArmInit() {

        lowMotor.set(ControlMode.PercentOutput, 0);
        lowMotor.configFactoryDefault();
        lowMotor.setNeutralMode(NeutralMode.Brake);
        lowMotor.setInverted(true);
        TalonFXConfiguration configs = new TalonFXConfiguration();
			
	    configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
			
	    lowMotor.configAllSettings(configs);
        lowMotor.setSelectedSensorPosition(0);

        upEncoder.setDistancePerPulse(Constants.encPulse);

    }
    public void ArmTeleop() {

        O1 = lowMotor.getSelectedSensorPosition()*lowDistPerTic;
        O2 = upEncoder.getDistance()*upDistPerTic;

        ExtendOne = 40.62*Math.cos(O1);
        ExtendTwo = 39.14*Math.cos(O1+O2);
        double ExtendedLength = ExtendOne + ExtendTwo - PivotToEdge;

        Extend1 = 40.62*Math.sin(O1);
        Extend2 = 39.14*Math.sin(O2);
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
            lowMotor.set(-.01);
        }
        else if (O2<177.5)
        {
            lowMotor.set(.01);
        }
        else
        {
            lowMotor.set(0);
        }

    }
}