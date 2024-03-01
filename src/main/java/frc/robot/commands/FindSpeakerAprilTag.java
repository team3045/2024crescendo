// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimeLightSub;
import frc.robot.subsystems.PositionerSub;

public class FindSpeakerAprilTag extends Command {
  private PositionerSub arm;
  private LimeLightSub vision;

  private boolean seen;
  /** Creates a new FindSpeakerAprilTag. */
  public FindSpeakerAprilTag(PositionerSub arm, LimeLightSub vision) {
    this.arm = arm;
    this.vision = vision;

    seen = false;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    seen = vision.getTargetSeen();
    if(seen || (seen &&Math.abs(vision.getTy()) < 1)){
      end(false);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    seen = vision.getTargetSeen();
    if(seen || (seen &&Math.abs(vision.getTy()) < 1)){
      end(false);
    }
    else{
      arm.decreaseAngle();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(seen||vision.getTargetSeen())
      return true;
    return false;
  }
}
