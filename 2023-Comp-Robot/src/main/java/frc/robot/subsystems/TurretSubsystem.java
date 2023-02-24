package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class TurretSubsystem extends SubsystemBase {
  
  private final CANSparkMax turretMotor  = new CANSparkMax(5, MotorType.kBrushed);

  private final DigitalInput clockLS = new DigitalInput(8);
  private final DigitalInput counterClockLS = new DigitalInput(9);

  public static Encoder turretEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k2X);

  double turretDistance = 0;

  double turretDistPerTic = 1/(((1024*10)*27)/360);

  boolean turretStop = false;
  boolean clockStop = false;
  boolean counterClockStop = false;

  // Stuff for limeLight
  public static boolean LowTarget = false;
  public static boolean HighTarget = false;

  public ShuffleboardTab tab = Shuffleboard.getTab("Vision");

  public GenericEntry limeLightXEntry = tab.add("LimeLight tx", 0).getEntry();
  public GenericEntry limeLightYEntry = tab.add("LimeLight ty", 0).getEntry();
  public GenericEntry limeLightAEntry = tab.add("LimeLight ta", 0).getEntry();

  public boolean enableLimelight = false;

  public boolean haveTurned = false;

  public void TurretInit(){

    turretEncoder.reset();
    turretEncoder.setDistancePerPulse(Constants.encPulse);
    turretMotor.setInverted(true);

  }

  public void TurretTeleop(){

    double rotate = RobotContainer.XCont2.getLeftX();
    rotate = Deadzone(rotate);

    if (RobotContainer.XCont2.getRightTriggerAxis() == 1)
    {
      enableLimelight = true;
    }
    if (RobotContainer.XCont2.getLeftBumperPressed() == true)
    {
      HighTarget = false;
      LowTarget = true;
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
    }
    if (RobotContainer.XCont2.getRightBumperPressed() == true)
    {
      LowTarget = false;
      HighTarget = true;
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1 );
    } 
    if (enableLimelight == true){
      Limelight_Tracking();
    }
    else if (turretStop == true)
    {
      TurretStop();
    }else
    {
      turretMotor.set(rotate);
    }
    if (turretEncoder.getDistance() != turretDistance)
    {
      turretDistance = turretEncoder.getDistance() * turretDistPerTic;
      System.out.println("Encoder Distance: "+turretDistance);
    }
    if (clockLS.get())
    {
      turretEncoder.reset();
      turretStop = true;
      clockStop = true;
    }
    else if (counterClockLS.get())
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
    if (value >= 0.05)
    {
      value = value*Constants.turretSpeed;
      return value;
    }
    /* Lower deadzone */
    else if (value <= -0.05)
    {
      value = value*Constants.turretSpeed;
      return value;
    }
    /* Outside deadzone */
      else{return 0;}
  }
  public void TurretStop(){
    if (turretStop == true && clockStop == true)
    {
      turretMotor.set(-.1);
    }
    /*else if (turretStop == true && counterClockStop == true)
    {
      turretMotor.set(.1);
    }*/
    else
    {
      turretMotor.set(0);
      turretStop = false;
      clockStop = false;
      counterClockStop = false;
    }
  }
  public void Limelight_Tracking()
  {
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

    double LimelightX = tx;
    double LimelightY = ty;
    double LimelightA = ta;
    limeLightXEntry.setDouble(LimelightX);
    limeLightYEntry.setDouble(LimelightY);
    limeLightAEntry.setDouble(LimelightA);

    double targetOffsetAngle_Vertical = ty;
    double limelightMountAngleDegrees = 19.875;
    double limelightLensHeightInches = 11;
    if (HighTarget == true){
      Constants.goalHeightInches = 43.9375;
    }
    else if (LowTarget == true){
      Constants.goalHeightInches = 24.00;
    }
   
    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

    Constants.distanceFromLimelightToGoalInches = (Constants.goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians)/* add distance from limelight to arm */;

    System.out.println("Distance to goal: "+Constants.distanceFromLimelightToGoalInches);
    
    double limelightEncoderVal = 0;

    if (haveTurned == false)
    {
      limelightEncoderVal = turretEncoder.getDistance() * turretDistPerTic;
      haveTurned = true;
    }

    if (limelightEncoderVal + 90 > turretEncoder.getDistance() * turretDistPerTic)
    {
      turretMotor.set(.1);
    }
    else if (Constants.havePlacedCone == false)
    {
      turretMotor.set(0);
      haveTurned = false;
      ArmSubsystem.ScoreCone();
    }
    else
    {
      turretMotor.set(0);
      haveTurned = false;
      Constants.havePlacedCone = false;
      enableLimelight = false;
    }
  }
}