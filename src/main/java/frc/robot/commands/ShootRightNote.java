// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PositionerSub;
import frc.robot.subsystems.ShooterSub;

public class ShootRightNote extends Command {
  private PositionerSub arm;
  private ShooterSub shooter;
  private int finishCount;

  private int count;
  /** Creates a new ShootClose. */
  public ShootRightNote(PositionerSub arm, ShooterSub shooter) {
    this.arm = arm;
    this.shooter = shooter;
    count = 0;
    finishCount = 0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm,shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.stopFeed();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    arm.goToRightNote();
    shooter.shootPct();
    count++;

    if(count >= 60){
      shooter.feed();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    count=0;
    finishCount = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    finishCount++;

    if(finishCount > 80){
      return true;
    }
    return false;
  }
}
