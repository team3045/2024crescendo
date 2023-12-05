// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.swervemath.math.Vector2;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int IMUCANid = 0;
    public static final class ArmConstants{
        public static final int firstStageMotorCANid = 0;
        public static final int secondStageMotorCANid1 = 0;
        public static final int secondStageMotorCANid2 = 0;
        public static final int intakeWristMotorCANid = 0;

        //inches and pounds
        public static final float firstStageNextPivot = 32;
        public static final float firstStageCenterOfMass = 15.67f;
        public static final float firstStageMass = 7.81f;

        public static final float secondStageNextPivot = 27.36f;
        public static final float secondStageCenterOfMass = 15.67f;
        public static final float secondStageMass = 4.80f;

        public static final float intakeCenterOfMass = 10.02f;
        public static final float intakeMass = 3.36f;

        public static final double firstStageKp = 0.0;
        public static final double firstStageKi = 0.0;
        public static final double firstStageKd = 0.0;

        public static final double secondStageKp = 0.0;
        public static final double secondStageKi = 0.0;
        public static final double secondStageKd = 0.0;

        public static final double intakeKp = 0.0;
        public static final double intakeKi = 0.0;
        public static final double intakeKd = 0.0;

        public static final double iLimit = 0.0;

        public static final double kSecondStageCurrentLimit = 35.0;

        public static final double firstStageGearRatio = 0;
        public static final double secondStageGearRatio = 0;
        public static final double intakeGearRatio = 0;

    }
}
