// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.limelightVision;

/** Add your docs here. */
public class FindCirclePoint {
    public Rotation2d robHeading;
    public Pose2d robPose;
    public Pose2d circlePoint;
    public double radius = limelightVision.getDistanceX();

    public FindCirclePoint(Swerve s_Swerve){
        robHeading = s_Swerve.getYaw();
        
    }
}
