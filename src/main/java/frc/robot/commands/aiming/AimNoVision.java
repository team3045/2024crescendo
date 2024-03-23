// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.aiming;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PositionerSub;
import frc.robot.subsystems.Swerve;

public class AimNoVision extends Command {
  private PositionerSub arm;
  private Swerve swerve;

  private Pose3d robotPose;
  private static final Pose3d speakerPose = DriverStation.getAlliance().get() == Alliance.Red ? 
    AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(3).get() : 
    AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(7).get(); 

  /** Creates a new AimNoVision. */
  public AimNoVision(PositionerSub arm, Swerve swerve) {
    this.arm = arm;
    this.swerve = swerve;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    robotPose = new Pose3d(swerve.getPose());
    Pose3d camPose = robotPose.transformBy(Constants.EstimationConstants.robotToCam);
    Transform3d difference = speakerPose.minus(camPose);
    double yDist = difference.getY();
    double norm = new Translation2d(difference.getX(), difference.getZ()).getNorm();

    double angle = Units.radiansToDegrees(Math.atan2(yDist,norm));
    arm.goToAngle(angle);

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
