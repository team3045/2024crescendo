// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.math.FindCirclePoint;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Retro;
import frc.robot.subsystems.Swerve;

public class AlineAlongCircle extends CommandBase {
  private Swerve s_Swerve;


  LimelightHelpers.LimelightResults lResults = LimelightHelpers.getLatestResults("");
  
  /** Creates a new AlineAlongCircle. */
  public AlineAlongCircle(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
  }

  public double yCalc(){
    try (PIDController yController = new PIDController(Constants.kPYGain, 0.1, 0)) {
      double yError = FindCirclePoint.findPose2d(s_Swerve).getY() - s_Swerve.getPose().getY();

      yError = Math.abs(yError) < 0.1 ? 0 : yError;

      return Math.abs(yController.calculate(yError)) < 0.1 ? 0 : yController.calculate(yError) * -1;

    }
  }

  public double xCalc(){
    try (PIDController xController = new PIDController(Constants.kPXGain, 0.1, 0)) {
      double xError = FindCirclePoint.findPose2d(s_Swerve).getX() - s_Swerve.getPose().getX();

      //xError = Math.abs(xError) < 0.1 ? 0 : xError;

      return Math.abs(xController.calculate(xError)) < 0.1 ? 0 : xController.calculate(xError) * -1;

    }
  }

  public double angCalc(){
    try(PIDController angController = new PIDController(Constants.kPAngleOffset, 0.1, 0)){
      double angError = FindCirclePoint.findPose2d(s_Swerve).getRotation().getDegrees() - s_Swerve.getYaw().getDegrees();

      if(Math.abs(angError) > 1){
        return angController.calculate(angError) *-1;
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


    return ChassisSpeeds.fromFieldRelativeSpeeds(-1 * xError / time, -1 * yError / time, -angError / time, s_Swerve.getYaw());
    //return new ChassisSpeeds(-1 * xError / time, -1 * yError / time, -angError / time);
  }

  public ChassisSpeeds splitIntoPoint(){
    ChassisSpeeds pointSpeeds = new ChassisSpeeds(calcWithSecs().vxMetersPerSecond, calcWithSecs().vyMetersPerSecond, 0);
    return pointSpeeds;
  }

  public ChassisSpeeds splitIntoAng(){
    ChassisSpeeds pointSpeeds = new ChassisSpeeds(0, 0, calcWithSecs().omegaRadiansPerSecond);
    return pointSpeeds;
  }

  public ChassisSpeeds difChassisSpeeds(){
    double xOutput = 0;
    double yOutput = 0;
    double angOutput = 0;

    try (PIDController xController = new PIDController(Constants.kPXGain, 0, 0)) {
      xOutput = xController.calculate(FindCirclePoint.distX(s_Swerve));
    }
    try (PIDController yController = new PIDController(Constants.kPYGain, 0, 0)) {
      yOutput = yController.calculate(FindCirclePoint.distY(s_Swerve));
    }
    try (PIDController angController = new PIDController(Constants.kPAngleOffset, 0, 0)) {
      angOutput = angController.calculate(FindCirclePoint.distAng(s_Swerve));
    }

    return new ChassisSpeeds(xOutput, yOutput, angOutput);
  }


  //Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    lResults = LimelightHelpers.getLatestResults("");

    //TRYBOTH
    //s_Swerve.driveAuto(calcWithSecs()); 
    //SmartDashboard.putNumber("VXpersec", -1 * xCalc());
    //SmartDashboard.putNumber("VYpersec", -1 * yCalc());
    //SmartDashboard.putNumber("AngPerSec", -angCalc());
    SmartDashboard.putString("CirclePoint", FindCirclePoint.findPose2d(s_Swerve).getTranslation().toString());
    SmartDashboard.putNumberArray("LimelightHelpers Pose", LimelightHelpers.getTargetPose_RobotSpace(""));
    SmartDashboard.putNumberArray("lResults", lResults.targetingResults.camerapose_robotspace);
    
    //s_Swerve.driveAuto(new ChassisSpeeds(xCalc(), yCalc(), angCalc()));
    //s_Swerve.driveAuto(difChassisSpeeds());
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
