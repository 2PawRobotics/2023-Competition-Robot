package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ArmSubsystem extends SubsystemBase {
    
    //private final CANSparkMax baseMotor = new CANSparkMax(5, MotorType.kBrushed);
    private final CANSparkMax upperArmMotor = new CANSparkMax(7, MotorType.kBrushed);

    public static Encoder baseEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k2X);

    public void TurretInit(){

        baseEncoder.setDistancePerPulse(Constants.encPulse);
        //TurretCenter();

    }

    public void TurretTeleop(){

        //double rotate = RobotContainer.rightJoy.getX();
        //rotate = Deadzone(rotate);
        //baseMotor.set(rotate);

    }

    /*public void TurretCenter(){
        if (baseEncoder.getDistance() < -1){
            baseMotor.set(.1);
        }
        else if ( baseEncoder.getDistance() > 1){
            baseMotor.set(-.1);
        }else{
            baseMotor.set(0);
        }
    }*/

    public void LowerArmInit(){

    }

    public void LowerArmTeleop(){

    }

    public void UpperArmInit(){

    }

    public void UpperArmTeleop(){
        double rotate = RobotContainer.rightJoy.getX();
        rotate = Deadzone(rotate);
            upperArmMotor.set(rotate);
    }

    public void ClowClose(){

    }

    public void ClawOpen(){

    }

    public double Deadzone(double value){
        /* Upper deadzone */
            if (value >= +0.05){
          value = value*Constants.testSpeed;
        return value;}
      
      /* Lower deadzone */
        else if (value <= -0.05){
          value = value*Constants.testSpeed;
        return value;}
      
      /* Outside deadzone */
        else{return 0;}
      }
}
