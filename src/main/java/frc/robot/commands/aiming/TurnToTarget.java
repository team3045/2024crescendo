// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.aiming;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.subsystems.AutoSub;
import frc.robot.subsystems.LimeLightSub;

public class TurnToTarget extends Command {
  private LimeLightSub vision;
  private Swerve swerve;

  private static double kP = 0.2;
  /** Creates a new TurnToTarget. */
  public TurnToTarget(LimeLightSub vision, Swerve swerve) {
    this.swerve =swerve;
    this.vision = vision;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(vision.getTargetSeen()){
      if(vision.getTx() < 2)
        CommandScheduler.getInstance().cancel(this);
      else {
        PIDController aController = new PIDController(kP, 0, 0);
        aController.setSetpoint(0);
        double aOutput = aController.calculate(vision.getTx());

        aController.setTolerance(1);

        swerve.driveTest(new ChassisSpeeds(0, 0, aOutput*0.7));

        aController.close();
      }
    }
    else{
      turnToAngle(getRotationGoal());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.driveField(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(vision.getTx())<1){
      return true;
    }
    return false;
  }

  public void turnToAngle(double goal){
    PIDController aController = new PIDController(0.1, 0, 0);
    aController.setSetpoint(goal);
    double aOutput = aController.calculate(swerve.getHeading().getDegrees());

    aController.setTolerance(1);

    System.out.println("turning GYRO");

    swerve.driveTest(new ChassisSpeeds(0, 0, aOutput));

    aController.close();
  }

  public double getRotationGoal(){
    Optional<Alliance> ally = DriverStation.getAlliance();
    if(ally.isPresent()){
      Pose3d tagPose;

      if(ally.get() == Alliance.Red)
        tagPose = Constants.EstimationConstants.fieldLayout.getTagPose(3).orElse(new Pose3d(swerve.getPose()));
      else
        tagPose = Constants.EstimationConstants.fieldLayout.getTagPose(5).orElse(new Pose3d(swerve.getPose()));

      Rotation2d goalRotation = tagPose.getRotation().toRotation2d().rotateBy(swerve.getHeading());
      return goalRotation.getDegrees();
    }
    else{
      System.out.println("DriverStation No Color");
      return swerve.getHeading().getDegrees();
    }
  }
}
