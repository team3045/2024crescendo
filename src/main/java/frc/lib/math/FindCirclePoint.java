// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.limelightVision;

/** Add your docs here. */
public class FindCirclePoint {
    public static Rotation2d robHeading;
    public static Pose2d robPose;
    public static Pose2d centerPoint;
    public static double radius = Constants.distanceDesired;


    public static Pose2d findPose2d(Swerve s_Swerve){

        radius = limelightVision.getDistanceX() * 12; //convert from inches to feet

        //radians b/c rot2d uses radians, 
        //adds the objangle to account for wherever obj is positioned on field
        robHeading = Rotation2d.fromDegrees(s_Swerve.getYaw().getDegrees());
        SmartDashboard.putNumber("robHeading", robHeading.getDegrees());
        //new pose that includes heading
        robPose = new Pose2d(new Translation2d(s_Swerve.getPose().getX(), s_Swerve.getPose().getY()), robHeading);

        double yOffset = s_Swerve.getPose().getY(); //have to do some logic for + or - later depending on whether the obj is to left or right
        double xOffset = s_Swerve.getPose().getX();

        double centerX = xOffset + Units.inchesToMeters(radius+Constants.Swerve.wheelBase/2+2) * Math.cos(90 - robHeading.getDegrees());

        double centerY =  Units.inchesToMeters(radius+Constants.Swerve.wheelBase/2+2) * Math.sin(90 - robHeading.getDegrees());

        centerY += robHeading.getDegrees() > 0 ? yOffset : -1 * yOffset;



        //subtract radius to find point on circle directly belox it
        return new Pose2d(new Translation2d(centerX, centerY), new Rotation2d(Math.toRadians(Constants.objAngle)));
    }

    public static double distY(Swerve s_Swerve){
        //radians b/c rot2d uses radians, 
        //adds the objangle to account for wherever obj is positioned on field
        robHeading = new Rotation2d(Math.toRadians(s_Swerve.getYaw().getDegrees() + Constants.objAngle)); 
        //new pose that includes heading
        robPose = new Pose2d(new Translation2d(s_Swerve.getPose().getX(), s_Swerve.getPose().getY()), robHeading);

        if(robHeading.getDegrees() > 180 || robHeading.getDegrees() < 0){
            return Units.inchesToMeters(radius+Constants.Swerve.wheelBase/2+2) * robHeading.getCos();
        }
        else
            return Units.inchesToMeters(radius+Constants.Swerve.wheelBase/2+2) * robHeading.getCos() * -1; 

    }

    public static double distX(Swerve s_Swerve){
        //radians b/c rot2d uses radians, 
        //adds the objangle to account for wherever obj is positioned on field
        robHeading = new Rotation2d(Math.toRadians(s_Swerve.getYaw().getDegrees() + Constants.objAngle)); 
        //new pose that includes heading
        robPose = new Pose2d(new Translation2d(s_Swerve.getPose().getX(), s_Swerve.getPose().getY()), robHeading);

        return Units.inchesToMeters(radius+Constants.Swerve.wheelBase/2+2) * robHeading.getSin();

    }

    public static double distAng(Swerve s_Swerve){
        return s_Swerve.getYaw().getDegrees() % 360;
    }
}
