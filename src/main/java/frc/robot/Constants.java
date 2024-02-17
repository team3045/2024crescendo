package frc.robot;

import java.sql.ClientInfoStatus;
import java.util.Map;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double startTime = System.currentTimeMillis();

    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 1;

        public static final COTSTalonFXSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
        COTSTalonFXSwerveConstants.SDS.MK4.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(28); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(26.5); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;
        public static final double driveBaseRadius = Math.sqrt(Math.pow((trackWidth / 2),2) + Math.pow(wheelBase / 2, 2));

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = 17.0; //chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /*Phoenix 6 Steer Motor gains */
        private static final Slot0Configs steerMotorGains = new Slot0Configs()
            .withKP(angleKP).withKI(angleKI).withKD(angleKD)
            .withKS(0).withKV(1.5).withKA(0);
        
        private static final Slot0Configs driveMotorGains = new Slot0Configs()
            .withKP(driveKP).withKI(driveKI).withKD(driveKD)
            .withKS(0).withKV(0).withKA(0);

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32 / 12; //TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51 / 12;
        public static final double driveKA = 0.27 / 12;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static double maxAngularVelocity = Math.PI * 2; //TODO: This must be tuned to specific robot

        public static boolean normalControl = true;

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /*The stator current at which the wheels start to slip;
        This needs to be tuned to your individual robot*/
        private static final double kSlipCurrentA = 300; //Amps, Read documentation for how to tune
        

        /*Not used right now but might try later to help with slipping
         * Based off CTRE phoenix 6 swerve with pathplanner example
         * also https://pro.docs.ctr-electronics.com/_/downloads/en/latest/pdf/
         */
        private static final SwerveModuleConstantsFactory constantsFactory = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(driveGearRatio)
            .withSteerMotorGearRatio(angleGearRatio)
            .withWheelRadius(2) // 4 inch wheels = 2 in radius
            .withSlipCurrent(angleCurrentLimit)
            .withSteerMotorGains(steerMotorGains)
            .withDriveMotorGains(driveMotorGains)
            .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
            .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
            .withSpeedAt12VoltsMps(maxSpeed) //Limited Max speed because we dont want it to drive at full power when pushed all the way
            .withSteerFrictionVoltage(0.25) //Simulated voltage necessary to overcome friction
            .withDriveFrictionVoltage(0.25) //Simulated voltage necessary to overcome friction
            .withFeedbackSource(SteerFeedbackType.RemoteCANcoder)
            .withSteerMotorInverted(true); //CounterClockWise positive is inverted i think?

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(119.88);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-120.41);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-9.1);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 0;
            public static final int canCoderID = 8;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-134.82);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3.0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1.0;
        public static final double kPYController = 1.0;
        public static final double kPThetaController = 1.0;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    //Used in LimelightSub
    public static final class EstimationConstants{

         public static final Pose3d originPose = new Pose3d(new Translation3d(0,0, 0), new Rotation3d(0,0,0));

        //where on ther field does the robot start compared to origin SET LATER
        public static final Transform3d originToRobotStart = new Transform3d(new Translation3d(Units.inchesToMeters(73), Units.inchesToMeters(37),0), new Rotation3d(0,0,0));
        public static final Transform3d testStartPose = new Transform3d(new Translation3d(5.0,5.0,0), new Rotation3d(0,0,0));
        
        public static final Pose3d robotStartPose = EstimationConstants.originPose.transformBy(originToRobotStart);

        //Cameras position in relation to robot SET LATER
        public static final Transform3d robotToCam = new Transform3d(new Translation3d(Swerve.wheelBase/2,0, Units.feetToMeters(1)), 
            new Rotation3d(0,0,0));

        //Where we want to be in relation to the tag
        public static final Transform3d tagToGoal = new Transform3d(new Translation3d(1, 0, 0), 
             new Rotation3d(0,0,Units.degreesToRadians(180)));

        //Max velocity and Max accel
        public static final TrapezoidProfile.Constraints X_CONSTRAINTS = new Constraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond,Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        public static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new Constraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond,Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        //Max angular velo and accel
        public static final TrapezoidProfile.Constraints A_CONSTRAINTS = new Constraints(Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond, Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared);

        //PID CONSTANTS TUNE LATER
        public static final double kPXGain = 0.5;
        public static final double kPYGain = 0.5;
        public static final double kPAGain = 1.0;

        //4.57, 2.57
        //SHOP: Map of Apriltags IDs and their 3d positions on the field SET LATER
        public static final Map<Double, Pose3d> idPoses = Map.of(
            3.0, new Pose3d(Units.inchesToMeters(0), Units.inchesToMeters(97.25), Units.inchesToMeters(57.25), new Rotation3d(0,0,Units.degreesToRadians(0))),
            5.0, new Pose3d(Units.inchesToMeters(180.25), Units.inchesToMeters(101.25), Units.inchesToMeters(53.25), new Rotation3d(0,0,Units.degreesToRadians(180))),
            11.0, new Pose3d(Units.inchesToMeters(116.75),Units.inchesToMeters(192.25),Units.inchesToMeters(51.25),new Rotation3d(0,0,Units.degreesToRadians(270))));
        
        //SHOP: Map of Apriltags IDs and their 3d positions on the field
        public static final AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    }
}
