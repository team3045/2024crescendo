// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class limelightVision extends SubsystemBase {
  private Swerve s_Swerve;

  private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight"); //Get specific Network Table
  private static double tX = table.getEntry("tx").getDouble(0); //targetOffsetAngle_Horizontal
  private static double tY = table.getEntry("ty").getDouble(0); //targetOffsetAngle_Vertical
  private static double tA= table.getEntry("ta").getDouble(0); //targetArea
  private static double tS = table.getEntry("ts").getDouble(0); //targetSkew
  private static double tV = table.getEntry("tV").getDouble(0); //if target is in view

  /** Creates a new limelightVision. */
  public limelightVision(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;

  }

  //TODO: 0 probably isnt a great value cause there will always be some sort of reflection
  //Change to some low value like a deadband on a controller
  public static boolean getTV(){
    return tV > 0 ? true : false; //if tV is greater than 0, target is in view
  }

  public static double getTX(){
    return tX;
  }

  public static double getTY(){
    return tY;
  }

  public static double getTA(){
    return tA;
  }

  public static double getTS(){
    return tS;
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("targetOffsetAngle_Horizontal: ", tX);
    SmartDashboard.putNumber("targetOffsetAngle_Vertical: ", tY);
    SmartDashboard.putNumber("targetArea: ", tA);
    SmartDashboard.putNumber("targetSkew: ", tS);
  }
}
