// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurretCommands;

import frc.robot.subsystems.TurretSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurretCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final TurretSubsystem armSubsystem;

  public TurretCommand(TurretSubsystem subsystem) {
    armSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    armSubsystem.TurretInit();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    armSubsystem.TurretTeleop();
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
