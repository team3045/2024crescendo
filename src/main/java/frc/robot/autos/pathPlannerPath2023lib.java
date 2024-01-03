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


    PathPlannerTrajectory loadedTraj = PathPlanner.loadPath("path2", Constants.AutoConstants.PATH_CONSTRAINTS);


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
