// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PoseEstimations;

public class PoseEstimator extends SubsystemBase {
  public SwerveDrivePoseEstimator sPoseEstimator;

  /** Creates a new PoseEstimator. */
  public PoseEstimator(Swerve s_Swerve) {

    //initialize pose estimator
    sPoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics,
     s_Swerve.getYaw(),
     s_Swerve.getModulePositions(),
     PoseEstimations.robotStartPose);
    
     sPoseEstimator.addVisionMeasurement(null, 0, null);


  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
