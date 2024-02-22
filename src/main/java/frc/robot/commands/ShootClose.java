// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PositionerSub;
import frc.robot.subsystems.ShooterSub;

public class ShootClose extends Command {
  private PositionerSub arm;
  private ShooterSub shooter;

  private int count;
  /** Creates a new ShootClose. */
  public ShootClose(PositionerSub arm, ShooterSub shooter) {
    this.arm = arm;
    this.shooter = shooter;
    count = 0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm,shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    shooter.shootPct();
    arm.goToSpeaker();
    count++;

    if(count >= 20){
      shooter.feed();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    count=0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}