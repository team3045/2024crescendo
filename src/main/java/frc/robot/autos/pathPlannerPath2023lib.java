// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class pathPlannerPath2023lib extends SequentialCommandGroup {
  /** Creates a new pathPlannerPath2023lib. */
  public pathPlannerPath2023lib(Swerve s_Swerve) {
    addRequirements(s_Swerve);


    PathPlannerTrajectory loadedTraj = PathPlanner.loadPath("spinSquare", Constants.AutoConstants.PATH_CONSTRAINTS);

    Pose2d firstPose = new Pose2d(0,0, new Rotation2d(Units.degreesToRadians(0)));
    Pose2d secondPose = new Pose2d(1,0, new Rotation2d(Units.degreesToRadians(0)));
    Pose2d thirdPose = new Pose2d(2,0, new Rotation2d(Units.degreesToRadians(90)));
    Pose2d fourthPose = new Pose2d(2,2, new Rotation2d(Units.degreesToRadians(180)));
    Pose2d fifthPose = new Pose2d(0,2, new Rotation2d(Units.degreesToRadians(-90)));
    Pose2d sixPose = new Pose2d(0,0, new Rotation2d(Units.degreesToRadians(0)));

    PathPoint firstPoint = new PathPoint(new Translation2d(0,0), new Rotation2d(Units.degreesToRadians(0)));
    PathPoint secondPoint = new PathPoint(new Translation2d(1,0), new Rotation2d(Units.degreesToRadians(0)));
    PathPoint thirdPoint = new PathPoint(new Translation2d(1.5,0), new Rotation2d(Units.degreesToRadians(90)));
    PathPoint fourthPoint = new PathPoint(new Translation2d(1.5,1.5), new Rotation2d(Units.degreesToRadians(180)));
    PathPoint fifthPoint = new PathPoint(new Translation2d(0,1.5), new Rotation2d(Units.degreesToRadians(-90)));
    PathPoint sixPoint = new PathPoint(new Translation2d(0,0), new Rotation2d(Units.degreesToRadians(0)));

    PathPlannerTrajectory firstTraj = PathPlanner.generatePath(
      Constants.AutoConstants.PATH_CONSTRAINTS,
      List.of(
        firstPoint,secondPoint,thirdPoint
      )
    );

    PathPlannerTrajectory secondTraj = PathPlanner.generatePath(
      Constants.AutoConstants.PATH_CONSTRAINTS,
      List.of(thirdPoint,fourthPoint)
    );

    PathPlannerTrajectory thirdTraj = PathPlanner.generatePath(
      Constants.AutoConstants.PATH_CONSTRAINTS,
      List.of(fourthPoint,fifthPoint)
    );

    PathPlannerTrajectory fourthTraj = PathPlanner.generatePath(
      Constants.AutoConstants.PATH_CONSTRAINTS,
      List.of(fifthPoint,sixPoint)
    );




    PPSwerveControllerCommand firstTrajCommand = new PPSwerveControllerCommand(
      firstTraj,
      s_Swerve::getPose,
      Constants.Swerve.swerveKinematics,
      new PIDController(Constants.AutoConstants.kPXController,0,0),
      new PIDController(Constants.AutoConstants.kPYController,0,0),
      new PIDController(Constants.AutoConstants.kPThetaController,0,0),
      s_Swerve::setModuleStates,
      false,
      s_Swerve
    );

    PPSwerveControllerCommand loadedTrajCommand = new PPSwerveControllerCommand(
      loadedTraj,
      s_Swerve::getPose,
      Constants.Swerve.swerveKinematics,
      new PIDController(Constants.AutoConstants.kPXController,0,0),
      new PIDController(Constants.AutoConstants.kPYController,0,0),
      new PIDController(Constants.AutoConstants.kPThetaController,0,0),
      s_Swerve::setModuleStates,
      false,
      s_Swerve
    );


    addCommands(
      new InstantCommand(() -> s_Swerve.resetOdometry(loadedTraj.getInitialHolonomicPose())),
      new InstantCommand(() -> s_Swerve.resetOdometry(loadedTraj.getInitialHolonomicPose())),
      loadedTrajCommand 
    );
  }
}
