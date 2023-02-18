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
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.TurretCommands.*;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  
  //Subsystems
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final TurretSubsystem armSubsystem = new TurretSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();

  //Drive Commands
  private final DriveCommand driveCommand = new DriveCommand(driveSubsystem);

  //Arm Command
  //private final UpperArmCommand upperArmCommand = new UpperArmCommand(armSubsystem);
  private final TurretCommand turretCommand = new TurretCommand(armSubsystem);
  private final LimelightCenterCommand limelightCenterCommand = new LimelightCenterCommand(armSubsystem);

  //LED Test Command
  private final LEDCommand ledCommand = new LEDCommand(ledSubsystem);

  //Gyro Commands
  private final BalanceCommand balanceCommand = new BalanceCommand(driveSubsystem);

  //Joysticks
  public static XboxController XCont = new XboxController(0);
  public static Joystick rightJoy = new Joystick(1);
  public static Joystick leftJoy = new Joystick(2);

  public RobotContainer() {
    
    CommandScheduler.getInstance().setDefaultCommand(driveSubsystem, driveCommand);
    //CommandScheduler.getInstance().setDefaultCommand(armSubsystem, upperArmCommand);
    CommandScheduler.getInstance().setDefaultCommand(ledSubsystem, ledCommand);
    CommandScheduler.getInstance().setDefaultCommand(armSubsystem, turretCommand);

    configureBindings();
  }

  private void configureBindings() {

    JoystickButton balanceButton = new JoystickButton(XCont, 2);
    balanceButton.whileTrue(balanceCommand);

    JoystickButton limelightCenterButton = new JoystickButton(leftJoy, 3);
    limelightCenterButton.whileTrue(limelightCenterCommand);

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
