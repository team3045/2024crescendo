// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PoseEstimations;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Swerve;

public class ChaseTagCommand extends CommandBase {
  private Swerve s_Swerve;
  private PhotonCamera camera;
  private int tagID;

  Pose2d previousPose;

  ProfiledPIDController xController;
  ProfiledPIDController yController;
  ProfiledPIDController aController;

  Pose3d targPose3d;
  Pose3d goalPose3d;
  Pose2d goalPose2d;

  PoseEstimator pEstimator;

  /** Creates a new ChaseTagCommand. */
  public ChaseTagCommand(Swerve swerve, PhotonCamera camera, int tagToChaseID) {
    this.s_Swerve = swerve;
    this.camera = camera;
    tagID = tagToChaseID;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController = new ProfiledPIDController(PoseEstimations.kPXGain, 0, 0, PoseEstimations.X_CONSTRAINTS);
    yController = new ProfiledPIDController(PoseEstimations.kPYGain, 0, 0, PoseEstimations.Y_CONSTRAINTS);
    aController = new ProfiledPIDController(PoseEstimations.kPAGain, 0, 0, PoseEstimations.A_CONSTRAINTS);

    aController.enableContinuousInput(-Math.PI, Math.PI);

    pEstimator = new PoseEstimator(s_Swerve, camera);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targPose3d = pEstimator.chaseTagPose3d(tagID);
    goalPose3d = targPose3d.transformBy(PoseEstimations.tagToGoal);
    goalPose2d = goalPose3d.toPose2d();

    xController.setGoal(goalPose2d.getX());
    yController.setGoal(goalPose2d.getY());
    aController.setGoal(goalPose2d.getRotation().getRadians());

    xController.setTolerance(0.1);
    yController.setTolerance(0.1);
    aController.setTolerance(Units.degreesToRadians(3));


    aController.calculate(pEstimator.getCurrentPose().getRotation().getRadians());

    s_Swerve.driveAuto(new ChassisSpeeds(
      xController.calculate(pEstimator.getCurrentPose().getX()),
      yController.calculate(pEstimator.getCurrentPose().getY()),
      aController.calculate(pEstimator.getCurrentPose().getRotation().getRadians())));

    Shuffleboard.getTab("Pose Estimations").add("GoalPose2d", goalPose2d);
    Shuffleboard.getTab("Pose Estimations").add("xOutput", xController.calculate(pEstimator.getCurrentPose().getX()));
    Shuffleboard.getTab("Pose Estimations").add("yOutput", yController.calculate(pEstimator.getCurrentPose().getY()));
    Shuffleboard.getTab("Pose Estimations").add("aOutput", aController.calculate(pEstimator.getCurrentPose().getRotation().getRadians()));

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
