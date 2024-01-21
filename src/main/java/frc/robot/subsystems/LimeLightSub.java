// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.constant.Constable;

import org.opencv.core.Mat;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.proto.Pose3dProto;
import edu.wpi.first.math.geometry.struct.Pose3dStruct;
import edu.wpi.first.math.geometry.struct.Rotation3dStruct;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EstimationConstants;

public class LimeLightSub extends SubsystemBase {

  //Network tables values

  private double pipelineNumber;

  public Pigeon2 gyro;

  private double[] camPose_TargetSpace;
  private Transform3d camPose_TargetSpaceTransform3d;

  private boolean targetSeen;
  private Pose3d robotPose;

  private double totalLatency;
  private double cl;
  private double tl;
  private double resultTimeStamp = 0;

  private double tx;

  private double tId;

  private static final ShuffleboardTab tab = Shuffleboard.getTab("limelight");
  private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  private GenericEntry seen, id, camPose, robPose, camTransform, test;


  /** Creates a new LimeLightSub. */
  public LimeLightSub() {
    targetSeen = table.getEntry("tv").getDouble(0) == 1.0;

    tx = table.getEntry("tx").getDouble(0);

    camPose_TargetSpace =
      table.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
    
    camPose_TargetSpaceTransform3d = new Transform3d(
      camPose_TargetSpace[0],
      camPose_TargetSpace[1],
      camPose_TargetSpace[2],
      new Rotation3d(0, 0, Units.degreesToRadians(tx)));

    

    tId = table.getEntry("tid").getDouble(-1);

    seen = tab.add("seen", getTargetSeen()).getEntry();
    id = tab.add("Tag ID", getID()).getEntry();
    camPose = tab.add("CamPose", pose3dToString(getCamPose())).getEntry();
    robPose = tab.add("RobPose", pose3dToString(getRobotPose())).getEntry();
    camTransform = tab.add("Target Cam Transform", transform3dToString(getCamToTargetTransform())).getEntry();
    test = tab.add("test", pose3dToString(
        transform(
          EstimationConstants.idPoses.get(11.0), 
          camPose_TargetSpaceTransform3d, true, true)
        )).getEntry();


    tl = table.getEntry("tl").getDouble(0);
    cl = table.getEntry("cl").getDouble(0);
    totalLatency = tl + cl;

  }

  //Getters

  public Pose3d getCamPose(){
    try {
      return EstimationConstants.idPoses.get(getID()).transformBy(camPose_TargetSpaceTransform3d);
    } catch (Exception e) {
      System.out.println("ID not found: " + getID());
    }

    return new Pose3d();
  }

  public Pose3d getRobotPose(){
    return getCamPose().transformBy(EstimationConstants.robotToCam.inverse());
  }

  public double getID(){
    return tId;
  }

  public boolean getTargetSeen(){
    return targetSeen;
  }

  public Transform3d getCamToTargetTransform(){
    return camPose_TargetSpaceTransform3d;
  }

  public double getTx(){
    return tx;
  }

  //Makes Pose3d into a string for shuffleboard
  public String pose3dToString(Pose3d pose){
    return 
      "Position: " +
      pose.getTranslation().toString() + " " +
      "Heading: " + 
      (Units.radiansToDegrees(pose.getRotation().getZ())%360);
  }

  //custom transform
  public Pose3d transform(Pose3d pose, Transform3d transform, boolean addX, boolean addZ){
    try {
      //double newX = !addX ? pose.getX() + transform.getX() : pose.getX() - transform.getX();
      //double newY = pose.getY() + transform.getY();
      //double newZ = !addZ ? pose.getZ() + transform.getZ() : pose.getZ() - transform.getZ();

      Pose3d temp = new Pose3d(transform.getTranslation(),new Rotation3d(0,0,0));
      temp = temp.rotateBy(new Rotation3d(Math.PI/2,Math.PI/2,Math.PI/2));
      transform = new Transform3d(temp.getTranslation(),temp.getRotation());

      SmartDashboard.putString("target Transform", transform.toString());


      double newX = pose.getX() + transform.getX();
      double newY = pose.getY() + transform.getY();
      double newZ = pose.getZ() + transform.getZ();


      return new Pose3d(newX,newY,newZ,new Rotation3d(0,0,pose.getRotation().getZ()-transform.getRotation().getZ()+Math.PI));
    } catch (Exception e) {
        return new Pose3d();
    }
  }

  public String transform3dToString(Transform3d transform3d){

    return 
      "X: " + transform3d.getX() +
      "Y: " + transform3d.getY() +
      "Z: " + transform3d.getZ() +
      "Heading: " + 
      Units.radiansToDegrees(transform3d.getRotation().getZ()); 
  }

  public double getTotalLatency(){
    return totalLatency;
  }

  public double getTimesStampMillis(){
    return System.currentTimeMillis() - Constants.startTime - totalLatency;
  }

  public Pose2d getVisionMeasurement(){
    try {
      Transform3d camToRobot = EstimationConstants.robotToCam.inverse();
      Pose3d tagPose = EstimationConstants.idPoses.get(getID());

      Transform3d tagToCam = getCamToTargetTransform().inverse();

      Translation3d fieldSystemTranslation = CoordinateSystem.convert(tagToCam.getTranslation().rotateBy(tagToCam.inverse().getRotation()), CoordinateSystem.EDN(), CoordinateSystem.NWU());

      System.out.println(fieldSystemTranslation.getY());
      System.out.println(tagPose.getZ() - fieldSystemTranslation.getY());
      

      //add .transformBy(camToRovot)
      Pose3d robotPose = tagPose.transformBy(new Transform3d(fieldSystemTranslation, new Rotation3d()));
    
      Pose2d visionPose = new Pose2d(robotPose.getX(),robotPose.getY(),new Rotation2d());

      System.out.println(visionPose.getY());

      return visionPose;

    } catch (Exception e) {
        System.out.println("ID NOT FOUND");
        return new Pose2d();
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    targetSeen = table.getEntry("tv").getDouble(0) == 1.0;

    tx = table.getEntry("tx").getDouble(0);
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

    seen.setBoolean(getTargetSeen());
    id.setDouble(getID());
    camPose.setString(pose3dToString(getCamPose()));
    robPose.setString(pose3dToString(getRobotPose()));
    camTransform.setString(transform3dToString(getCamToTargetTransform()));
    test.setString(
      pose3dToString(
        transform(
          EstimationConstants.idPoses.get(11.0), 
          getCamToTargetTransform(),true,true)
        )
    );
    SmartDashboard.putString("test2", getVisionMeasurement().toString());
    
  }


}
