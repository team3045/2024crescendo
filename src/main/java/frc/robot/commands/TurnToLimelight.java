// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.limelightVision;

public class TurnToLimelight extends Command {
  private Swerve swerve;
  /** Creates a new TurnToLimelight. */
  public TurnToLimelight(Swerve swerve) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.swerve = swerve;

    addRequirements(this.swerve);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error = limelightVision.getTX();
    
    if(Math.abs(error) > 1)
      swerve.turnToAngle(error*Constants.kPAngleOffset*-1);
    else
      swerve.driveAuto(new ChassisSpeeds(0, 0, 0));
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
