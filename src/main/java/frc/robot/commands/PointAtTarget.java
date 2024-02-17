// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.LimeLightSub;
import frc.robot.subsystems.PositionerSub;

public class PointAtTarget extends Command {
  private Swerve swerve;
  private LimeLightSub vision;
  private PositionerSub arm;

  /** Creates a new PointAtTarget. */
  public PointAtTarget(Swerve swerve, LimeLightSub vision, PositionerSub arm) {
    this.vision = vision;
    this.swerve = swerve;
    this.arm = arm;

    addRequirements(swerve, arm);
  }

  /*Regression to calculate angle based on distance */
  public double calcAngle(){
    double distance = vision.getHorizDistanceSpeaker();

    /*break out of function if target not seen */
    if(distance == -1){
      System.out.println("target not seen, cant move arm");
      return -1;
    }

    /*Regression for angle, currently arbitray values */
    double angle = 3 * Math.pow(distance, 3);
    angle += 2 * Math.pow(distance, 2);
    angle += 1.5 * distance;
    angle += 5;

    return angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.goToAngle(calcAngle());
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
