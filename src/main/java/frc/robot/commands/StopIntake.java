// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.Intake;

public class StopIntake extends Command {
  private Intake intake;
  private ShooterSub shooter;

  /** Creates a new StopIntaKE. */
  public StopIntake(Intake intake, ShooterSub shooter) {
    this.intake = intake;
    this.shooter = shooter;

    addRequirements(shooter,intake);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.disable();
    shooter.stopFeed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
