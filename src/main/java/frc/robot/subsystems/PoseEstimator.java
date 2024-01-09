// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.Timestamp;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PoseEstimations;


public class PoseEstimator extends SubsystemBase {

  private final PhotonCamera limelightCam;
  public SwerveDrivePoseEstimator sPoseEstimator;

  private PhotonPipelineResult previousResult;
  private double previousTimestamp = 0;

  private boolean hadTarget = false;

  private Swerve s_Swerve;
  

  /** Creates a new PoseEstimator. */
  public PoseEstimator(Swerve s_Swerve, PhotonCamera camera) {

    this.s_Swerve = s_Swerve;

    //initialize pose estimator
    sPoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics,
      s_Swerve.getYaw(),
      s_Swerve.getModulePositions(),
      PoseEstimations.robotStartPose);


    limelightCam = camera; 

    //can set unique standard vision deviations CHECK LATER
    //sPoseEstimator.setVisionMeasurementStdDevs(null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    PhotonPipelineResult lResult = limelightCam.getLatestResult();
    double resulTimestamp = lResult.getTimestampSeconds(); 

    if(resulTimestamp != previousTimestamp && lResult.hasTargets()){
      previousResult = lResult;
      previousTimestamp = previousResult.getTimestampSeconds();

      PhotonTrackedTarget target = lResult.getBestTarget();
      int fiducialID = target.getFiducialId();

      Shuffleboard.getTab("Pose Estimations").add("Target ID", target.getFiducialId());

      if(target.getPoseAmbiguity() <= 0.2 && PoseEstimations.idPoses.containsKey(fiducialID)){
        Pose3d targetPose = PoseEstimations.idPoses.get(fiducialID);
        Transform3d camToTarget = target.getBestCameraToTarget();
        Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

        Pose3d robotPose = camPose.transformBy(PoseEstimations.robotToCam.inverse());
        sPoseEstimator.addVisionMeasurement(robotPose.toPose2d(), resulTimestamp);
      }
    }

    sPoseEstimator.updateWithTime(Timer.getFPGATimestamp(), s_Swerve.getYaw(), s_Swerve.getModulePositions());
      
    Shuffleboard.getTab("Pose Estimations").add("Pose Estimator Pose", getCurrentPose().toString());
    Shuffleboard.getTab("Pose Estimations").add("Target Visible", lResult.hasTargets());
    Shuffleboard.getTab("Pose Estimations").add("Latest TimeStamp", resulTimestamp);
    Shuffleboard.getTab("Pose Estimations").add("Photon Rsults", lResult.toString());
  }

  public Pose2d getCurrentPose(){
    return sPoseEstimator.getEstimatedPosition();
  }

}
