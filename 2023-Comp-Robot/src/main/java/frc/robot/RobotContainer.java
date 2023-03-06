// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.ArmCommands.ArmAutonCommand;
import frc.robot.commands.ArmCommands.ArmCommand;
import frc.robot.commands.ArmCommands.CubeScoreCommand;
import frc.robot.commands.ClawCommands.ClawAutonCommand;
import frc.robot.commands.ClawCommands.ClawCommand;
import frc.robot.commands.DriveCommands.*;
import frc.robot.commands.TurretCommands.*;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
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
  private final ClawSubsystem clawSubsystem = new ClawSubsystem();
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final TurretSubsystem turretSubsystem = new TurretSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();

  //Drive Commands
  private final DriveCommand driveCommand = new DriveCommand(driveSubsystem);
  private final DriveAutonCommand driveAutonCommand = new DriveAutonCommand(driveSubsystem);
  private final DriveAutonScoreCommand driveAutonScoreCommand = new DriveAutonScoreCommand(driveSubsystem);

  //Claw Commands
  private final ClawCommand clawCommand = new ClawCommand(clawSubsystem);
  private final ClawAutonCommand clawAutonCommand = new ClawAutonCommand(clawSubsystem);

  //Arm Commands
  private final ArmCommand armCommand = new ArmCommand(armSubsystem);
  private final CubeScoreCommand cubeScoreCommand = new CubeScoreCommand(armSubsystem);
  private final ArmAutonCommand armAutonCommand = new ArmAutonCommand(armSubsystem);

  //Turret Commands
  private final TurretCommand turretCommand = new TurretCommand(turretSubsystem);
  private final LimelightCenterCommand limelightCenterCommand = new LimelightCenterCommand(turretSubsystem);

  //LED Commands
  private final LEDCommand ledCommand = new LEDCommand(ledSubsystem);

  //Gyro Commands
  private final BalanceCommand balanceCommand = new BalanceCommand(driveSubsystem);

  //Joysticks
  public static XboxController XCont = new XboxController(0);
  public static XboxController XCont2 = new XboxController(1);

  public RobotContainer() {
    
    CommandScheduler.getInstance().setDefaultCommand(driveSubsystem, driveCommand);
    CommandScheduler.getInstance().setDefaultCommand(armSubsystem, armCommand);
    CommandScheduler.getInstance().setDefaultCommand(ledSubsystem, ledCommand);
    CommandScheduler.getInstance().setDefaultCommand(turretSubsystem, turretCommand);
    CommandScheduler.getInstance().setDefaultCommand(clawSubsystem, clawCommand);

    configureBindings();
  }

  private void configureBindings() {

    JoystickButton balanceButton = new JoystickButton(XCont, 3);
    balanceButton.whileTrue(balanceCommand);

    JoystickButton limelightCenterButton = new JoystickButton(XCont2, 6);
    limelightCenterButton.whileTrue(limelightCenterCommand);

    JoystickButton cubeButton = new JoystickButton(XCont2, 11);
    cubeButton.whileTrue(cubeScoreCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    String autoSelected = Constants.sendChooser.getSelected();
    if (autoSelected == Constants.scoreDriveAuto){
      return new ClawAutonCommand(clawSubsystem)
      .alongWith(new DriveAutonScoreCommand(driveSubsystem), new ArmAutonCommand(armSubsystem));
    }
    else if (autoSelected == Constants.balanceAuto)
    {
      return new DriveAutonBalanceCommand(driveSubsystem);
    }
    else if (autoSelected == Constants.scoreBalanceAuto)
    {
      return new DriveAutonScoreCommand(driveSubsystem)
      .alongWith( new ArmAutonCommand(armSubsystem));
    }
    else
    {
      return new DriveAutonCommand(driveSubsystem);
    }
  }
}
