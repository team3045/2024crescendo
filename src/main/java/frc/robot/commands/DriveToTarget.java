// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EstimationConstants;
import frc.robot.subsystems.LimeLightSub;
import frc.robot.subsystems.Swerve;

public class DriveToTarget extends Command {
  private double targetID;
  private LimeLightSub vision;
  private Swerve swerve;

  private Pose3d targetPose3d;
  private Pose2d targetPose2d;

  private Pose2d goalPose2d;

  private static final double kPXGain = 0.1;
  private static final double kPAGain = 0.1;

  private static final Map<Double, Transform2d> offsets = Map.of(
    11.0, new Transform2d(new Translation2d(0.0,2.0),Rotation2d.fromDegrees(180))
    );
  
  /** Creates a new DriveToTarget. */
  public DriveToTarget(double targetID, LimeLightSub vision, Swerve swerve) {
    this.targetID = targetID;
    this.vision = vision;
    this.swerve = swerve;

    targetPose3d = EstimationConstants.idPoses.get(targetID);
    targetPose2d = targetPose3d.toPose2d();

    goalPose2d = targetPose2d.transformBy(offsets.get(targetID));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xOutput = 0;
    double yOutput = 0;
    double aOutput = 0;

    PIDController xController = new PIDController(kPXGain, 0, 0);
    PIDController yController = new PIDController(kPXGain, 0, 0);
    PIDController aController = new PIDController(kPAGain, 0, 0);

    xController.setSetpoint(goalPose2d.getX());
    yController.setSetpoint(goalPose2d.getY());
    aController.setSetpoint(vision.getID() == targetID ? 0 : targetPose2d.getRotation().getDegrees()+180);
    

    xOutput = xController.calculate(swerve.getPose().getX());
    yOutput = yController.calculate(swerve.getPose().getY());
    aOutput = aController.calculate(vision.getID() == targetID ? vision.getTx() : swerve.getYaw().getDegrees());

    swerve.driveTest(new ChassisSpeeds(xOutput, yOutput, aOutput));

    xController.close();
    yController.close();
    aController.close();
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
