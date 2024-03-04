// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EstimationConstants;
import frc.robot.commands.FeedAndShoot;
import frc.robot.commands.FullAim;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.ShootClose;
import frc.robot.commands.ShootMiddleNote;
import frc.robot.commands.ShootRightNote;

public class AutoSub extends SubsystemBase {
  private Swerve swerve;
  private LimeLightSub aimingVision;
  private boolean instantiated = false;

  /*For On Fly PathPlanning */
  //Map of where we want to be in relation to each AprilTag
  private static final Map<Double, Transform2d> offsets = Map.of(
    5.0, new Transform2d(new Translation2d(2.5,0),Rotation2d.fromDegrees(180)));
  //Specific constraints for onFly pathplanning
  private final PathConstraints onFlyConstraints = new PathConstraints(4.0, 1.0, Math.PI*3, Math.PI * 4);



  /** Creates a new AutoSub. */
  public AutoSub(Swerve swerve, PositionerSub positionerSub, ShooterSub shooterSub, Intake intake, LimeLightSub aimiingVision) {
    this.swerve = swerve;
    this.aimingVision = aimiingVision;

    instantiated = true;

    NamedCommands.registerCommand("ShootClose", new ShootClose(positionerSub, shooterSub));
    NamedCommands.registerCommand("IntakeNote",  new IntakeNote(intake, shooterSub, positionerSub));
    NamedCommands.registerCommand("FullAim", new FullAim(positionerSub, aimingVision, shooterSub, swerve, null));
    NamedCommands.registerCommand("FeedAndShoot", new FeedAndShoot(shooterSub));
    NamedCommands.registerCommand("MiddleNote", new ShootMiddleNote(positionerSub, shooterSub));
    NamedCommands.registerCommand("Stop Feed", new InstantCommand(() -> shooterSub.stopFeed()));
    NamedCommands.registerCommand("RightNote", new ShootRightNote(positionerSub, shooterSub));

    PIDConstants translationConstants = new PIDConstants(55,0,0);
    PIDConstants rotationConstants = new PIDConstants(50,0,0);
    HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(translationConstants,rotationConstants,3.0, Constants.Swerve.driveBaseRadius, new ReplanningConfig(true,true));
    AutoBuilder.configureHolonomic(swerve::getPose, swerve::setPose, swerve::getChassisSpeeds, swerve::driveField, config,() -> false, swerve);
  }


  //get an Autonomous command from a pathplanner path
  public Command getAutoCommand(String autoName){
    //PathPlannerPath path = PathPlannerPath.fromPathFile("2 Note Right");

    //return AutoBuilder.followPath(path);
    return AutoBuilder.buildAuto(autoName);
  }

  //gets the pose where you want to be in relation to an apriltag
  public Optional<Pose2d> getGoalPose(double targetId, LimeLightSub vision){
    Pose2d goalPose = null;

    try {
      Pose3d targetPose3d = EstimationConstants.idPoses.get(targetId);
      Pose2d targetPose2d = targetPose3d.toPose2d();

      goalPose = targetPose2d.transformBy(offsets.get(targetId));
    } catch (Exception e) {
      System.out.println("ID not found Get Goal Pose");
    }

    return Optional.of(goalPose);
  }


  //intended usage by passing getGoalPose into this as param
  //returns command to go to that pose
  public Command getOnFlyCommand(Pose2d goalPose){
    return AutoBuilder.pathfindToPose(goalPose,onFlyConstraints,0.0,0.0);
  }
  

  public void turnToTarget(LimeLightSub vision){
    if(vision.getTargetSeen()){
      PIDController aController = new PIDController(0.1, 0, 0);
      aController.setSetpoint(0);
      double aOutput = aController.calculate(vision.getTx());

      aController.setTolerance(1);

      System.out.println("turning");

      swerve.driveTest(new ChassisSpeeds(0, 0, aOutput*0.7));

      aController.close();
    }
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
