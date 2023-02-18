package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class TurretSubsystem extends SubsystemBase {
  
  private final WPI_TalonFX turretMotor  = new WPI_TalonFX(5, "rio");

  private final DigitalInput clockLS = new DigitalInput(6);
  private final DigitalInput counterClockLS = new DigitalInput(7);

  public static Encoder turretEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k2X);

  double turretDistance = 0;

  boolean turretStop;
  boolean clockStop;
  boolean counterClockStop;

  public void TurretInit(){

    turretEncoder.setDistancePerPulse(Constants.encPulse);
    //TurretCenter();

  }

  public void TurretTeleop(){

    double rotate = RobotContainer.leftJoy.getX();
    rotate = Deadzone(rotate)*.5;

    if (turretStop == true)
    {
      TurretStop();
    }
    else
    {
      turretMotor.set(rotate);
    }
    if (turretEncoder.getDistance() != turretDistance)
    {
      turretDistance = turretEncoder.getDistance();
      System.out.println("Encoder Distance: "+turretDistance);
    }
    if (clockLS.get() == true)
    {
      turretEncoder.reset();
      turretStop = true;
      clockStop = true;
    }
    else if (counterClockLS.get() == true)
    {
      turretStop = true;
      counterClockStop = true;
    }
  }

  public void CenterLimelight(){

    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

    if (tx > 1)
    {
      turretMotor.set(.1);
    }
    else if (tx < -1)
    {
      turretMotor.set(-.1);
    }
    else
    {
      turretMotor.set(0);
    }
  }

  public double Deadzone(double value){
    /* Upper deadzone */
    if (value >= +0.09)
    {
      value = value*Constants.testSpeed;
      return value;
    }
    /* Lower deadzone */
    else if (value <= -0.09)
    {
      value = value*Constants.testSpeed;
      return value;
    }
    /* Outside deadzone */
      else{return 0;}
  }
  public void TurretStop(){
    turretMotor.set(0);
    if (turretStop == true && clockStop == true)
    {
      turretMotor.set(-.01);
    }
    else if (turretStop == true && counterClockStop == true)
    {
      turretMotor.set(.01);
    }
    else
    {
      turretMotor.set(0);
      turretStop = false;
      clockStop = false;
      counterClockStop = false;
    }
  }
}
