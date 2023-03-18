// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArmCommands.ArmAutonCommand;
import frc.robot.commands.ArmCommands.ArmCommand;
import frc.robot.commands.ClawCommands.ClawAutonCommand;
import frc.robot.commands.ClawCommands.ClawCommand;
import frc.robot.commands.DriveCommands.*;
import frc.robot.commands.TurretCommands.*;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  
  //Subsystems
  private final ClawSubsystem clawSubsystem = new ClawSubsystem();
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final TurretSubsystem turretSubsystem = new TurretSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();

  //Drive Commands
  private final DriveCommand driveCommand = new DriveCommand(driveSubsystem);

  //Claw Commands
  private final ClawCommand clawCommand = new ClawCommand(clawSubsystem);

  //Arm Commands
  private final ArmCommand armCommand = new ArmCommand(armSubsystem);

  //Turret Commands
  private final TurretCommand turretCommand = new TurretCommand(turretSubsystem);

  //Gyro Commands
  private final BalanceCommand balanceCommand = new BalanceCommand(driveSubsystem);

  //Joysticks
  public static XboxController XCont = new XboxController(0);
  public static XboxController XCont2 = new XboxController(1);

  public RobotContainer() 
  {
    CommandScheduler.getInstance().setDefaultCommand(driveSubsystem, driveCommand);
    CommandScheduler.getInstance().setDefaultCommand(armSubsystem, armCommand);
    CommandScheduler.getInstance().setDefaultCommand(turretSubsystem, turretCommand);
    CommandScheduler.getInstance().setDefaultCommand(clawSubsystem, clawCommand);

    configureBindings();
  }

  private void configureBindings()
  {
    JoystickButton balanceButton = new JoystickButton(XCont, 3);
    balanceButton.whileTrue(balanceCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    String autoSelected = Constants.sendChooser.getSelected();
    if (autoSelected == Constants.scoreDriveAuto)
    {
      return new ClawAutonCommand(clawSubsystem)
      .alongWith(new DriveAutonScoreCommand(driveSubsystem), new ArmAutonCommand(armSubsystem));
    }
    else if (autoSelected == Constants.balanceAuto)
    {
      return new DriveAutonBalanceCommand(driveSubsystem);
    }
    else if (autoSelected == Constants.scoreDriveAuto2)
    {
      return new DriveAutonScoreCommand2(driveSubsystem)
      .alongWith(new ArmAutonCommand(armSubsystem), new ClawAutonCommand(clawSubsystem));
    }
    else if (autoSelected == Constants.balanceScoreAuto)
    {
      return new DriveAutonScoreBalanceCommand(driveSubsystem)
      .alongWith(new ArmAutonCommand(armSubsystem), new ClawAutonCommand(clawSubsystem));
    }
    else
    {
      return new DriveAutonCommand(driveSubsystem);
    }
  }
}
