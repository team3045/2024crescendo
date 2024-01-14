// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PoseEstimations;

public class LimelightSub extends SubsystemBase {

  private double[] camPose_TargetSpace;

  private Transform3d camPose_TargetSpaceTransform3d;

  private boolean targetSeen;

  private Pose3d robotPose;

  private double tId;

  private static final ShuffleboardTab tab = Shuffleboard.getTab("Pose Estimations");

  /** Creates a new LimelightSub. */
  public LimelightSub() {

    targetSeen = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1.0;

    camPose_TargetSpace = 
      NetworkTableInstance.getDefault().getTable("limelight").
        getEntry("camerapose_targetspace").getDoubleArray(new double[6]);

    camPose_TargetSpaceTransform3d = new Transform3d(
      camPose_TargetSpace[0], 
      camPose_TargetSpace[1], 
      camPose_TargetSpace[2], 
      new Rotation3d(
        camPose_TargetSpace[3],
        camPose_TargetSpace[4],
        camPose_TargetSpace[5]
    ));

    if(targetSeen){
      tId = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(-1);
    }
  }

  //Getters 
  //all getters also update values

  public double getTagID() {
    tId = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(-1);
    return tId;
  }


  public Pose3d getCamPose(){

    camPose_TargetSpace = 
      NetworkTableInstance.getDefault().getTable("limelight").
        getEntry("camerapose_targetspace").getDoubleArray(new double[6]);

    camPose_TargetSpaceTransform3d = new Transform3d(
      camPose_TargetSpace[0], 
      camPose_TargetSpace[1], 
      camPose_TargetSpace[2], 
      new Rotation3d(
        camPose_TargetSpace[3],
        camPose_TargetSpace[4],
        camPose_TargetSpace[5]
    ));

    return PoseEstimations.idPoses.get(getTagID()).transformBy(camPose_TargetSpaceTransform3d);
  }

  public Pose3d getRobPose(){
    return getCamPose().transformBy(PoseEstimations.robotToCam.inverse());
  }

  public boolean getTargetSeen(){
    targetSeen = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1.0;
    return targetSeen;
  }

  @Override
  public void periodic() {

      tab.add("Target Seen", getTargetSeen());
      tab.add("Tag ID", getTagID());
      tab.add("CamPose", getCamPose());
      tab.add("RobPose", getRobPose());  
    
  }
}
