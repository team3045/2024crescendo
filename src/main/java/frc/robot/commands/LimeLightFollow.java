// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.print.attribute.standard.ReferenceUriSchemesSupported;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
<<<<<<< HEAD
=======
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
>>>>>>> testingBranch1-1-24
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.limelightVision;

public class LimeLightFollow extends CommandBase {
  private boolean targetSeen;
  private double tX;
<<<<<<< HEAD
  private double prevTX;
  private double tY;
  private Swerve s_Swerve;
  private boolean inPosition;
=======
  private Swerve s_Swerve;
  private boolean inPosition;
  double error;
>>>>>>> testingBranch1-1-24
  double angleOffset;

  /** Creates a new LimeLightFollow. */
  public LimeLightFollow(Swerve s_Swerve) {
    double area = limelightVision.getTA();

    //make sure there is acc a target in view
    if (limelightVision.getTV())
      targetSeen = area > Constants.areaThreshold ? true: false;

    tX = limelightVision.getTX();
<<<<<<< HEAD
    prevTX = tX;
=======
>>>>>>> testingBranch1-1-24

    this.s_Swerve = s_Swerve;

    if(targetSeen && limelightVision.getTX() < 1)
      inPosition = true;
    else 
      inPosition = false;
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.s_Swerve);
  }

  public double distanceOutput(){
<<<<<<< HEAD
    PIDController disController = new PIDController(Constants.kPXGain, 0, 0);
    double error = limelightVision.getDistanceX() - Constants.distanceDesired;

    return disController.calculate(error) * -1;
  }

  public double rotationOutput(){
    double error = limelightVision.getTX();
    try (PIDController rController = new PIDController(Constants.kPAngleOffset, 0, 0)) {
      if(Math.abs(error) > 3)
        return rController.calculate(error);
=======
    try (PIDController disController = new PIDController(Constants.kPXGain, 0, 0)) {
      double error = limelightVision.getDistanceX() - Constants.distanceDesired;
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
>>>>>>> testingBranch1-1-24
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
