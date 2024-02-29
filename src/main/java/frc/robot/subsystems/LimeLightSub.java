// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EstimationConstants;

public class LimeLightSub extends SubsystemBase {
  //Camera name
  private String name;

  //Network tables values
  public Pigeon2 gyro;

  private double[] camPose_TargetSpace;
  private Transform3d camPose_TargetSpaceTransform3d;

  private boolean targetSeen;

  private double totalLatency;
  private double cl;
  private double tl;

  private double tx;
  private double ty;

  private double tId;

  private static final ShuffleboardTab localizerTab = Shuffleboard.getTab("limelight");
  private NetworkTable table;

  private GenericEntry seen, id, camTransform, test;

  private static final int speakerPipline = 1;
  private static final int localizationPipeline = 0;


  /** Creates a new LimeLightSub. */
  public LimeLightSub(String name) {
    this.name = name;
    table = NetworkTableInstance.getDefault().getTable(this.name);

    targetSeen = table.getEntry("tv").getDouble(0) == 1.0;

    tx = table.getEntry("tx").getDouble(0);
    ty = table.getEntry("ty").getDouble(0);

    camPose_TargetSpace =
      table.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
    
    camPose_TargetSpaceTransform3d = new Transform3d(
      camPose_TargetSpace[0],
      camPose_TargetSpace[1],
      camPose_TargetSpace[2],
      new Rotation3d(0, 0, Units.degreesToRadians(tx)));

    

    tId = table.getEntry("tid").getDouble(-1);

    seen = localizerTab.add("seen by: " + name, getTargetSeen()).getEntry();
    id = localizerTab.add("Tag ID by: " + name, getID()).getEntry();
    camTransform = localizerTab.add("Target Cam Transform by: " + name, transform3dToString(getCamToTargetTransform())).getEntry();


    tl = table.getEntry("tl").getDouble(0);
    cl = table.getEntry("cl").getDouble(0);
    totalLatency = tl + cl;

  }

  //Getters

  //returns ID of currently seen target, return -1 if none seen
  //uses of this method often are accompanied by try catch in case the value is not in a map

  public double getID(){
    return tId;
  }

  public boolean getTargetSeen(){
    return targetSeen;
  }


  //returns camPose in target space 
  //Coord System X: East, Y: Down, Z: North , From target POV
  public Transform3d getCamToTargetTransform(){
    return camPose_TargetSpaceTransform3d;
  }

  //Horizontal angle between camera and target center
  //if target is to the left of camera center tx is neg, vice versa is pos 
  public double getTx(){
    return tx;
  }

  public double getTy(){
    return ty;
  }

  //Used with getTimeStampMillis to add vision estimations at the correct time
  public double getTotalLatency(){
    return totalLatency;
  }

  //We record the System start time at beginning to get a single time reference for entire codebase
  public double getTimesStampMillis(){
    return System.currentTimeMillis() - Constants.startTime - totalLatency;
  }

  public double getTimeStampSeconds(){
    return getTimesStampMillis() * 100;
  }

  public double getNorm(){
    return getCamToTargetTransform().getTranslation().getNorm();
  }

  //END Getters

  //Formatting

  //Makes Pose3d into a string for shuffleboard
  public String pose3dToString(Pose3d pose){
    return 
      "Position: " +
      pose.getTranslation().toString() + " " +
      "Heading: " + 
      (Units.radiansToDegrees(pose.getRotation().getZ())%360);
  }

  public String transform3dToString(Transform3d transform3d){

    return 
      "X: " + transform3d.getX() +
      "Y: " + transform3d.getY() +
      "Z: " + transform3d.getZ() +
      "Heading: " + 
      Units.radiansToDegrees(transform3d.getRotation().getZ()); 
  }

  //End Formatting

  /*Returns negative one if ID not seen */
  public double getHorizDistanceSpeaker(){
    table.getEntry("pipeline").setNumber(speakerPipline);
    if(getTargetSeen() && getID() == 3){
      //gets second value of the array which should be depth, or z, in camera space
      //essentially is horizontal distance to target, no matter what angle facing bc camera space
      double distance = table.getEntry("targetpose_cameraspace").getDoubleArray(new double[] {-1.0,-1.0,-1.0,-1.0,-1.0,-1.0})[2];
      table.getEntry("pipeline").setNumber(localizationPipeline);
      return distance;
    }
    else{
      table.getEntry("pipeline").setNumber(localizationPipeline);
      return -1;
    }
  }

  public Transform3d getTargetCamTransform3d(){
    return getCamToTargetTransform().inverse();
  }

  public double getNormToSpeaker(){
    return getTargetCamTransform3d().getTranslation().getNorm();
  }

  //returns vision measurement to add to pose estimator
  public Pose2d getVisionMeasurement(){
    try {
      //should be the inverse but for some reason this way works?
      Transform3d camToRobot = EstimationConstants.robotToCam;
      Pose3d tagPose = EstimationConstants.idPoses.get(getID());

      Transform3d tagToCam = getCamToTargetTransform().inverse();

      Translation3d fieldSystemTranslation = CoordinateSystem.convert(tagToCam.getTranslation().rotateBy(tagToCam.inverse().getRotation()), CoordinateSystem.EDN(), CoordinateSystem.NWU());
      
      //add .transformBy(camToRovot)
      Pose3d robotPose = tagPose.transformBy(new Transform3d(fieldSystemTranslation, new Rotation3d())).transformBy(camToRobot);
    
      Pose2d visionPose = new Pose2d(robotPose.getX(),robotPose.getY(),new Rotation2d());

      return visionPose;

    } catch (Exception e) {
       // System.out.println("ID NOT FOUND");
        return new Pose2d();
    }
  }

  public void setAimingPipeline(){
    table.getEntry("pipeline").setNumber(1);
  }

  public void setLocalizerPipeline(){
    table.getEntry("pipeline").setNumber(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //Update Values every loop
    targetSeen = table.getEntry("tv").getDouble(0) == 1.0;

    tx = table.getEntry("tx").getDouble(0);
    ty = table.getEntry("ty").getDouble(0);
    camPose_TargetSpace =
      table.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
    
    camPose_TargetSpaceTransform3d = new Transform3d(
      camPose_TargetSpace[0],
      camPose_TargetSpace[1],
      camPose_TargetSpace[2],
      new Rotation3d(0,0,Units.degreesToRadians(tx)));

    tId = table.getEntry("tid").getDouble(-1);

    tl = table.getEntry("tl").getDouble(0);
    cl = table.getEntry("cl").getDouble(0);

    totalLatency = tl + cl;

    //end Updates

    //Update shuffleboard
    seen.setBoolean(getTargetSeen());
    id.setDouble(getID());
    camTransform.setString(transform3dToString(getCamToTargetTransform()));

    SmartDashboard.putString("test2", getVisionMeasurement().toString());
    
  }
}
