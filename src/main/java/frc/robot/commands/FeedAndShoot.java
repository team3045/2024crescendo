// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PositionerSub;
import frc.robot.subsystems.ShooterSub;

public class FeedAndShoot extends Command {
  private ShooterSub shooter;
  private PositionerSub arm;

  /** Creates a new FeedAndShoot. */
  public FeedAndShoot(ShooterSub shooter, PositionerSub arm) {
    this.shooter = shooter;
    addRequirements(shooter);
    this.arm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.freeze();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double bottomSpeed = ShooterSub.MPSToRPS(29, 4*Math.PI);
    // double topSpeed = ShooterSub.MPSToRPS(30, 4*Math.PI);
    // shooter.shootSpeed(topSpeed, bottomSpeed);
    shooter.shootPct();
    
    if(shooter.getCurrentSpeedMPS() > 29.5){
      shooter.feed();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Timer.delay(0.2);
    arm.unfreeze();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
