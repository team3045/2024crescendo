// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class FieldSwerveCommand extends Command {

  private Swerve s_Swerve;
  private Supplier<Rotation2d> robotAngleSupplier;
  private DoubleSupplier translationXSupplier;
  private DoubleSupplier translationYSupplier;
  private DoubleSupplier rotationXSupplier;

  private final SlewRateLimiter translateXRateLimiter = new SlewRateLimiter(6.0);
  private final SlewRateLimiter translateYRateLimiter = new SlewRateLimiter(6.0);
  private final SlewRateLimiter rotationRateLimiter = new SlewRateLimiter(5*Math.PI);

  /** Creates a new FieldSwerveCommand. */
  public FieldSwerveCommand(
    Swerve s_Swerve, 
    Supplier<Rotation2d> robotAngleSupplier, 
    DoubleSupplier translationXSupplier,
    DoubleSupplier translationYSupplier,
    DoubleSupplier rotationXSupplier) {

    this.s_Swerve = s_Swerve;
    this.robotAngleSupplier = robotAngleSupplier;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationXSupplier = rotationXSupplier;

    addRequirements(s_Swerve);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Rotation2d robotAngle = robotAngleSupplier.get();

    // Calculate field relative speeds
    ChassisSpeeds chassisSpeeds = s_Swerve.getChassisSpeeds();
    ChassisSpeeds robotSpeeds = new ChassisSpeeds(
        chassisSpeeds.vxMetersPerSecond * robotAngle.getCos() - chassisSpeeds.vyMetersPerSecond * robotAngle.getSin(),
        chassisSpeeds.vyMetersPerSecond * robotAngle.getCos() + chassisSpeeds.vxMetersPerSecond * robotAngle.getSin(),
        chassisSpeeds.omegaRadiansPerSecond);

     // Reset the slew rate limiters, in case the robot is already moving
     translateXRateLimiter.reset(robotSpeeds.vxMetersPerSecond);
     translateYRateLimiter.reset(robotSpeeds.vyMetersPerSecond);
     rotationRateLimiter.reset(robotSpeeds.omegaRadiansPerSecond);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    s_Swerve.driveTest(
      ChassisSpeeds.fromFieldRelativeSpeeds(
        translationXSupplier.getAsDouble(),
        translationYSupplier.getAsDouble(),
        rotationXSupplier.getAsDouble(),
        robotAngleSupplier.get())
    );

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //stop robot
    s_Swerve.driveTest(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
