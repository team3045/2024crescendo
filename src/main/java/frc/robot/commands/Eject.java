// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.PositionerSub;
import frc.robot.subsystems.ShooterSub;

public class Eject extends Command {
  private ShooterSub shooter;
  private PositionerSub arm;
  private int count;
  /** Creates a new Eject. */
  public Eject(PositionerSub arm, ShooterSub shooter) {
    this.shooter = shooter;
    this.arm = arm;

    count = 0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm,shooter);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.goToVertical();
    if(count >= 60)
      shooter.eject();
    count++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopAll();
    count = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
