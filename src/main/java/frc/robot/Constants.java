package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.autos.pathPlannerPath2023lib;

public final class Constants {
    public static final double stickDeadband = 0.05;
    private static final double WHEEL_DIAMETER = Units.inchesToMeters(4); //in inches
    private static final double MAX_MOTOR_UNITS_PER_SECOND = 225300;
    private static final double kMotorUnitsToRotations = 2048;
    private static final double kRotationToRadians = 2*Math.PI;
    private static final double kAngularVelocityToWheelSpeed = WHEEL_DIAMETER*Math.PI;
    public static final double rotationModifier = 0.04; //reduce the speed of rotation on teleop
    public static final double translationModifier = 0.65; //reduce the speed of translation on teleop

    public static final double objectHeight = 23.5; //in inches
    public static final double limelightHeight = 18.5; //in inches
    public static final double objDiff = (objectHeight - limelightHeight) / 12; //in feet
    public static final double kPAngleOffset = 0.15;
    public static final double kPXGain = 1.0;
    public static final double distanceDesired = 36;
    public static final double areaThreshold = 0.1;
    public static final double objAngle = 0; //what angle it is at in relation to our robot zero heading
    public static final double kPYGain = 0.15;


    public static final class Swerve {

        public static final int pigeonID = 15;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /*public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);*/

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(27); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(32); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = Units.inchesToMeters(4*Math.PI); //set to wheel circumfrance

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = 10.20; //set to drive gear ratio
        public static final double angleGearRatio = 15.43; //set to angle gear ratio

        /* Motor Inverts */
        public static final boolean angleMotorInvert = true; //set if angle motor inverted
        public static final boolean driveMotorInvert = false; //set if drive motor inverted

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false; //set if canCOder inverted

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        /*public static final double angleKP = driveRatios.angleKP;//set later
        public static final double angleKI = driveRatios.angleKI; //setr later
        public static final double angleKD = driveRatios.angleKD;//set later
        public static final double angleKF = driveRatios.angleKF;//set later*/
        public static final double angleKP = 0.1;//set later
        public static final double angleKI = 0; //setr later
        public static final double angleKD = 0;//set later
        public static final double angleKF = 0; 
        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        /*public static final double driveKS = (0.041263 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (2.3303/ 12);
        public static final double driveKA = (0.073661 / 12)*/

        public static final double driveKS = (0.32/ 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51/ 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Radians per Second */
        public static final double maxAngularVelocity = MAX_MOTOR_UNITS_PER_SECOND / kMotorUnitsToRotations * kRotationToRadians; //TODO: This must be tuned to specific robot
        /** Meters per Second */
        public static final double maxSpeed = Units.inchesToMeters(maxAngularVelocity*kAngularVelocityToWheelSpeed); //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(313.15);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(144.22);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(286.9);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 0;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(198);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = Constants.Swerve.maxSpeed / 4;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Constants.Swerve.maxAngularVelocity / 10;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI / 4;
    
        public static final double kPXController = 1.3;
        public static final double kPYController = 1.3;
        public static final double kPThetaController = 2.3;

        public static final double autonForwardDistance = 2/*meters*/;
        public static final double autonSidewaysDistance = 0;

        public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(
            kMaxSpeedMetersPerSecond, 
            kMaxAccelerationMetersPerSecondSquared 
           );
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
