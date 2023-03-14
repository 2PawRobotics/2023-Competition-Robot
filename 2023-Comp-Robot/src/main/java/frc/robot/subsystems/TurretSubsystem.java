package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class TurretSubsystem extends SubsystemBase {
  
  private final static CANSparkMax turretMotor  = new CANSparkMax(5, MotorType.kBrushed);

  private final DigitalInput clockLS = new DigitalInput(8);
  private final DigitalInput counterClockLS = new DigitalInput(9);

  public static Encoder turretEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k2X);

  double turretDistance = 0;

  static double turretDistPerTic = 1/(((1024*10)*27)/360);

  boolean turretStop = false;
  boolean clockStop = false;
  boolean counterClockStop = false;

  double turretAngle = 0;

  boolean turretBoolean = false;

  public void TurretInit()
  {
    if (turretBoolean == false)
    {
      turretEncoder.reset();
      turretEncoder.setDistancePerPulse(turretDistPerTic);
      turretMotor.setInverted(true);
      turretMotor.setIdleMode(IdleMode.kBrake);
      turretBoolean = true;
    }
  }

  public void TurretTeleop()
  {
    turretAngle = turretEncoder.get()/(((1024*9)*27)/360);
    double rotate = RobotContainer.XCont2.getRightX();
    rotate = Deadzone(rotate);

    if (turretStop == true)
    {
      TurretStop();
    }
    else
    {
      turretMotor.set(rotate);
    }
    if (clockLS.get() == false)
    {
      turretEncoder.reset();
      turretStop = true;
      clockStop = true;
    }
    else if (counterClockLS.get() == false)
    {
      turretStop = true;
      counterClockStop = true;
    }
    else
    {
      turretStop = false;
      clockStop = false;
      counterClockStop = false;
    }
  }

  public double Deadzone(double value)
  {
    if (value >= 0.1)
    {
      value = value*Constants.turretSpeed;
      return value;
    }
    else if (value <= -0.1)
    {
      value = value*Constants.turretSpeed;
      return value;
    }
    else
    {
      return 0;
    }
  }

  public void TurretStop()
  {
    if (turretStop == true && clockStop == true)
    {
      turretMotor.set(-.1);
    }
    else if (turretStop == true && counterClockStop == true)
    {
      turretMotor.set(.1);
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