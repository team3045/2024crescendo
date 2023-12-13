// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.print.attribute.standard.ReferenceUriSchemesSupported;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.limelightVision;

public class LimeLightFollow extends CommandBase {
  private boolean targetSeen;
  private double tX;
  private double prevTX;
  private double tY;
  private Swerve s_Swerve;
  private boolean inPosition;
  double angleOffset;

  /** Creates a new LimeLightFollow. */
  public LimeLightFollow(Swerve s_Swerve, limelightVision lVision) {
    double area = limelightVision.getTA();
    if (limelightVision.getTV())
      targetSeen = area > Constants.areaThreshold ? true: false;

    tX = limelightVision.getTX();
    prevTX = tX;

    this.s_Swerve = s_Swerve;
    if(targetSeen && limelightVision.getTX() < 1)
      inPosition = true;
    else 
      inPosition = false;
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.s_Swerve, lVision);
  }

  public double distanceOutput(){
    PIDController distController = new PIDController(Constants.kPYGain, 0, 0);

    return distController.calculate(
      (Constants.objectHeight / Math.tan(limelightVision.getTY())),
      Constants.distanceDesired
    );
  }

  public double rotationOutput(double angleOffset){
    PIDController rotController = new PIDController(Constants.kPAngleOffset, 0, 0);

    prevTX = tX;    
    tX = limelightVision.getTX() + angleOffset;
    if (targetSeen && !inPosition)
      return rotController.calculate(s_Swerve.getYaw().getDegrees(), tX);
    else if (targetSeen && inPosition)
      return 0;
    else
      return rotController.calculate(s_Swerve.getYaw().getDegrees(), prevTX);
    
    
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
    s_Swerve.driveAuto(new ChassisSpeeds(0,distanceOutput(), rotationOutput(angleOffset)));
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
