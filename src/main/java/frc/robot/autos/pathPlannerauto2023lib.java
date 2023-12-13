// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;

import javax.swing.plaf.metal.MetalIconFactory.FolderIcon16;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
public class pathPlannerauto2023lib extends SequentialCommandGroup {
  /** Creates a new pathPlannerauto2023lib. */
  public pathPlannerauto2023lib(Swerve s_Swerve) {
    addRequirements(s_Swerve);

    Pose2d firstPose = new Pose2d(0,0,new Rotation2d(Units.degreesToRadians(0)));
    Pose2d secondPose = new Pose2d(1,0,new Rotation2d(Units.degreesToRadians(90)));
    Pose2d thirdPose = new Pose2d(2,0,new Rotation2d(Units.degreesToRadians(180)));
    Pose2d fourthPose = new Pose2d(2,2,new Rotation2d(Units.degreesToRadians(-90)));
    Pose2d fifthPose = new Pose2d(0,2,new Rotation2d(Units.degreesToRadians(0)));

    PathPoint firstPoint = new PathPoint(new Translation2d(0, 0), new Rotation2d(Units.degreesToRadians(0)));
    PathPoint secondPoint = new PathPoint(new Translation2d(1, 0), new Rotation2d(Units.degreesToRadians(0)));
    PathPoint thirdPoint = new PathPoint(new Translation2d(2, 0), new Rotation2d(Units.degreesToRadians(90)));
    PathPoint fourthPoint = new PathPoint(new Translation2d(2, 2), new Rotation2d(Units.degreesToRadians(180)));
    PathPoint fifthPoint = new PathPoint(new Translation2d(0, 2), new Rotation2d(Units.degreesToRadians(-90)));
    PathPoint sixPoint = new PathPoint(new Translation2d(0, 0), new Rotation2d(Units.degreesToRadians(0)));


    PathPlannerTrajectory firstTraj = PathPlanner.generatePath(
      Constants.AutoConstants.PATH_CONSTRAINTS,
      List.of(
        firstPoint,secondPoint,thirdPoint,fourthPoint,fifthPoint,sixPoint
      ));

    PPHolonomicDriveController firstTrajController = new PPHolonomicDriveController(
      new PIDController(Constants.AutoConstants.kPXController, 0, 0),
      new PIDController(Constants.AutoConstants.kPYController, 0, 0), 
      new PIDController(Constants.AutoConstants.kPThetaController, 0, 0));

    PPSwerveControllerCommand firstTrajcommand = new PPSwerveControllerCommand(
      firstTraj,
      s_Swerve::getPose,
      Constants.Swerve.swerveKinematics,
      new PIDController(Constants.AutoConstants.kPXController, 0, 0),
      new PIDController(Constants.AutoConstants.kPYController, 0, 0),
      new PIDController(Constants.AutoConstants.kPThetaController, 0, 0),
      s_Swerve::setModuleStates,
      false,
      s_Swerve);


    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> s_Swerve.resetOdometry(firstTraj.getInitialHolonomicPose())),
      new InstantCommand(() -> s_Swerve.zeroGyro()),
      firstTrajcommand
      
    );
  }
}
