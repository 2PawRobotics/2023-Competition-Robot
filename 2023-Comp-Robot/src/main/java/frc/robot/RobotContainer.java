// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ArmCommands.*;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  
  //Subsystems
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();

  //Drive Commands
  private final DriveCommand driveCommand = new DriveCommand(driveSubsystem);

  //Claw Commands
  private final ClawOpenCommand clawOpenCommand = new ClawOpenCommand(armSubsystem);
  private final ClawCloseCommand clawCloseCommand = new ClawCloseCommand(armSubsystem);

  //Turret Commands
  private final TurretCommand turretCommand = new TurretCommand(armSubsystem);

  //Lower Arm Commands
  private final LowerArmCommand lowerArmCommand = new LowerArmCommand(armSubsystem);

  //Upper Arm Commands
  private final UpperArmCommand upperArmCommand = new UpperArmCommand(armSubsystem);


  //Joysticks
  public static XboxController XCont = new XboxController(0);
  public static Joystick rightJoy = new Joystick(1);
  //public static Joystick rightJoy = new Joystick(2);

  public RobotContainer() {
    
    CommandScheduler.getInstance().setDefaultCommand(driveSubsystem, driveCommand);
    //CommandScheduler.getInstance().setDefaultCommand(armSubsystem, turretCommand);
    //CommandScheduler.getInstance().setDefaultCommand(armSubsystem, lowerArmCommand);
    CommandScheduler.getInstance().setDefaultCommand(armSubsystem, upperArmCommand);

    configureBindings();
  }

  private void configureBindings() {

    //Claw Commands
    /*JoystickButton clawCloseButton = new JoystickButton(leftJoy, 1);
    clawCloseButton.onTrue(clawCloseCommand);
    JoystickButton clawOpenButton = new JoystickButton(leftJoy, 2);
    clawOpenButton.onTrue(clawOpenCommand);*/

    //Turret Commands
    

    //Lower Arm Commands


    //Upper Arm Commands

  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    String autoSelected = Constants.sendChooser.getSelected();
    if (autoSelected == Constants.customAuto){
      return null;
    }
    else{
      return null;
    }
  }
}
