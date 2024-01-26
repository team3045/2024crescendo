// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.Console;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.Constants.EstimationConstants;
import frc.robot.subsystems.LimeLightSub;
import frc.robot.subsystems.Swerve;

public class OnFlyPathPlanner {
  private Swerve swerve;
  private LimeLightSub vision;
  private Pose2d targetPose;

  private static final Map<Double, Transform2d> offsets = Map.of(
    5.0, new Transform2d(new Translation2d(1.5,0),Rotation2d.fromDegrees(180))
    );
  
  private final PathConstraints constraints = new PathConstraints(4.0, 1.0, Math.PI*3, Math.PI * 4);

  /** Creates a new OnFlyPathPlanner. */
  public OnFlyPathPlanner(Swerve swerve, LimeLightSub vision, double targetId) {
    this.swerve = swerve;
    this.vision = vision;
    
    Pose3d targetPose3d = EstimationConstants.idPoses.get(targetId);
    Pose2d targetPose2d = targetPose3d.toPose2d();

    this.targetPose = targetPose2d.transformBy(offsets.get(targetId));

    PIDConstants translationConstants = new PIDConstants(1);
    PIDConstants rotatiConstants = new PIDConstants(1);
    ReplanningConfig replanningConfig = new ReplanningConfig();

    HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(
      translationConstants, 
      rotatiConstants,
    4.0, //Max speed of individual module
      Constants.Swerve.driveBaseRadius, 
      replanningConfig);

    AutoBuilder.configureHolonomic(
      swerve::getPose,
      swerve::resetPose, 
      swerve::getChassisSpeeds, //get current robot relative chassisspeeds
      swerve::driveTest, //drive robot relative with chassisspeeds
      config, 
      () -> { //if we are red then flip, otherwise false
        var alliance = DriverStation.getAlliance();

        if(alliance.isPresent()){
          return alliance.get() == DriverStation.Alliance.Red;
        }

        return false;
      },
      swerve); //subsytem to act on

  }

  public Command getPPCommand(){
    return AutoBuilder.pathfindToPose(targetPose,constraints,0.0,0.0);
  }
  public void actPPCommand(){
    getPPCommand();
  }
}
