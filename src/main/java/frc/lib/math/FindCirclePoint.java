// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.limelightVision;

/** Add your docs here. */
public class FindCirclePoint {
    public static Rotation2d robHeading;
    public static Pose2d robPose;
    public static Pose2d centerPoint;
    public static double radius = limelightVision.getDistanceX();


    public static Pose2d findPose2d(Swerve s_Swerve){

        //radians b/c rot2d uses radians, 
        //adds the objangle to account for wherever obj is positioned on field
        robHeading = new Rotation2d(Math.toRadians(s_Swerve.getYaw().getDegrees() + Constants.objAngle)); 
        //new pose that includes heading
        robPose = new Pose2d(new Translation2d(s_Swerve.getPose().getX(), s_Swerve.getPose().getY()), robHeading);

        double centerX = radius * robHeading.getSin();

        double centerY = radius * robHeading.getCos();


        //subtract radius to find point on circle directly belox it
        return new Pose2d(new Translation2d(centerX - radius, centerY), new Rotation2d(Math.toRadians(Constants.objAngle)));
    }
}
