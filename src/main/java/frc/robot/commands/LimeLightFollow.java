// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.print.attribute.standard.ReferenceUriSchemesSupported;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.limelightVision;

public class LimeLightFollow extends CommandBase {
  private boolean targetSeen;
  private double tX;
  private Swerve s_Swerve;
  private boolean inPosition;
  double error;
  double angleOffset;

  /** Creates a new LimeLightFollow. */
  public LimeLightFollow(Swerve s_Swerve) {
    double area = limelightVision.getTA();

    //make sure there is acc a target in view
    if (limelightVision.getTV())
      targetSeen = area > Constants.areaThreshold ? true: false;

    tX = limelightVision.getTX();

    this.s_Swerve = s_Swerve;

    if(targetSeen && limelightVision.getTX() < 1)
      inPosition = true;
    else 
      inPosition = false;
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.s_Swerve);
  }

  public double distanceOutput(){
    try (PIDController disController = new PIDController(Constants.kPXGain, 0, 0)) {
      double error = limelightVision.getDistanceX() - Constants.distanceDesired / 12; //inches to feet
      error = Math.abs(error) < 0.1 ? 0 : error;

      return (Math.abs(disController.calculate(error)*-1)) < 0.1 ? 0 : error;
    }
    
  }

  public double rotationOutput(){
    error = limelightVision.getTX();
    try (PIDController rController = new PIDController(Constants.kPAngleOffset, 0, 0)) {
      if(Math.abs(error) > 1){
        return rController.calculate(error);
      }
      else
        return 0;
    }
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleOffset = s_Swerve.getYaw().getDegrees();
    s_Swerve.driveAuto(new ChassisSpeeds(0,0,0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Swerve.driveAuto(new ChassisSpeeds(distanceOutput(),0, rotationOutput()));
    SmartDashboard.putNumber("LimeLight_out_distance", distanceOutput());
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
