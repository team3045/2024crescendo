// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;

import javax.crypto.spec.RC2ParameterSpec;
import javax.swing.TransferHandler.TransferSupport;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.xPosition;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class pathPlannerAuto extends SequentialCommandGroup {
  /** Creates a new pathPlannerAuto. */
  public pathPlannerAuto(Swerve s_Swerve) {
    ReplanningConfig replanConfig = new ReplanningConfig();

    Pose2d firstPose = new Pose2d(new Translation2d(0,0), new Rotation2d(0));
    Pose2d middlePose = new Pose2d(new Translation2d(1,0), new Rotation2d(0));
    Pose2d secondPose = new Pose2d(new Translation2d(2,0), new Rotation2d(0));
    Pose2d thirdPose = new Pose2d(new Translation2d(2,2), new Rotation2d(0));
    Pose2d fourthPose = new Pose2d(new Translation2d(0,2), new Rotation2d(0));


    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(List.of(firstPose, middlePose,secondPose,thirdPose,fourthPose));

    PathPoint pointOne = new PathPoint(new Translation2d(0,0), new Rotation2d(0));
    PathPoint pointTwo = new PathPoint(new Translation2d(1,0), new Rotation2d(0));

    HolonomicPathFollowerConfig holonomicConfig = new HolonomicPathFollowerConfig(
      new PIDConstants(Constants.AutoConstants.kPYController,0,0), 
      new PIDConstants(Constants.AutoConstants.kPThetaController, 0, 0), 
      Constants.AutoConstants.kMaxSpeedMetersPerSecond,
      Constants.Swerve.wheelBase/2,replanConfig);

    /*PathPlannerPath firstPath = new PathPlannerPath(
      List.of(new Translation2d(0,0), new Translation2d(2,0)),
      Constants.AutoConstants.PATH_CONSTRAINTS,
      Constants.AutoConstants.GOAL_END_STATE);*/

    PathPlannerPath testPath = new PathPlannerPath(bezierPoints, 
      Constants.AutoConstants.PATH_CONSTRAINTS, 
      Constants.AutoConstants.GOAL_END_STATE, false);


    /*FollowPathHolonomic followFirstPath = new FollowPathHolonomic(
      firstPath, 
      s_Swerve::getPose, s_Swerve::getChassisSpeeds, s_Swerve::driveAuto, holonomicConfig, s_Swerve);*/

    FollowPathHolonomic followTestPath = new FollowPathHolonomic(
      testPath, 
      s_Swerve::getPose, s_Swerve::getChassisSpeeds, s_Swerve::driveAuto, holonomicConfig, s_Swerve);



    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> s_Swerve.resetOdometry(testPath.getPreviewStartingHolonomicPose())),
      followTestPath
      //new xPosition(s_Swerve)
    );
  }
}
