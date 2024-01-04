// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.math.FindCirclePoint;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AlineAlongCircle extends CommandBase {
  private Swerve s_Swerve;

  /** Creates a new AlineAlongCircle. */
  public AlineAlongCircle(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
  }

  public double yCalc(){
    try (PIDController yController = new PIDController(Constants.kPYGain, 0, 0)) {
      double yError = FindCirclePoint.findPose2d(s_Swerve).getY() - s_Swerve.getPose().getY();

      yError = Math.abs(yError) < 0.1 ? 0 : yError;

      return Math.abs(yController.calculate(yError)) < 0.1 ? 0 : yController.calculate(yError) * -1;

    }
  }

  public double xCalc(){
    try (PIDController xController = new PIDController(Constants.kPXGain, 0, 0)) {
      double xError = FindCirclePoint.findPose2d(s_Swerve).getY() - s_Swerve.getPose().getY();

      xError = Math.abs(xError) < 0.1 ? 0 : xError;

      return Math.abs(xController.calculate(xError)) < 0.1 ? 0 : xController.calculate(xError) * -1;

    }
  }

  public double angCalc(){
    try(PIDController angController = new PIDController(Constants.kPAngleOffset, 0, 0)){
      double angError = FindCirclePoint.findPose2d(s_Swerve).getRotation().getDegrees() - s_Swerve.getYaw().getDegrees();

      if(Math.abs(angError) > 1){
        return angController.calculate(angError);
      }
      else
        return 0;
    }
  }

  public ChassisSpeeds calcWithSecs(){
    double xError = FindCirclePoint.findPose2d(s_Swerve).getY() - s_Swerve.getPose().getY();
    double yError = FindCirclePoint.findPose2d(s_Swerve).getY() - s_Swerve.getPose().getY();
    double angError = FindCirclePoint.findPose2d(s_Swerve).getRotation().getDegrees() - s_Swerve.getYaw().getDegrees();

    double time = 2; //How many seconds, in this case it will complete the motion in two seconds

    return new ChassisSpeeds(xError / time, yError / time, angError / time);


  }

  //Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    //TRYBOTH
    
    //s_Swerve.driveAuto(calcWithSecs()); 

    s_Swerve.driveAuto(new ChassisSpeeds(xCalc(), yCalc(), angCalc()));
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
