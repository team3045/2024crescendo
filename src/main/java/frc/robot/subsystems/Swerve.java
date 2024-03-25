package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.lib.util.ModifiedSignalLogger;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.EstimationConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.stream.IntStream;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.struct.Pose2dStruct;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import static edu.wpi.first.units.Units.*;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveDrivePoseEstimator mPoseEstimator;
    public LimeLightSub[] limeLightSubs;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    private ChassisSpeeds dChassisSpeeds;
    private SysIdRoutine sysIdRoutine;

     /**
    * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
    * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
    */
    //ie how much it trusts its current estimate
    private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.4, 0.4, Units.degreesToRadians(3));

    /**
    * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
    * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
    */
    //how much it trusts the vision measurements
     private static Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.9, 0.9, 999999);

     private final MutableMeasure<Voltage> appliedVoltage = MutableMeasure.mutable(Volts.of(0));
     private final MutableMeasure<Distance> appliedDistance = MutableMeasure.mutable(Meters.of(0));
     private final MutableMeasure<Velocity<Distance>> appliedVelocity = MutableMeasure.mutable(MetersPerSecond.of(0));

     private final Measure<Velocity<Voltage>> rampRate = Volts.of(0.5).per(Seconds.of(1));

    public Swerve(LimeLightSub[] limeLightSubs) {
        this.limeLightSubs = limeLightSubs;

        gyro = new Pigeon2(Constants.Swerve.pigeonID, "Canivore 3045");
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        mPoseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
             getGyroYaw(), 
             getModulePositions(), 
             EstimationConstants.robotStartPose.toPose2d(),
             stateStdDevs,
             visionMeasurementStdDevs);

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions(),EstimationConstants.robotStartPose.toPose2d());

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(rampRate,null,null,null),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> driveVolts(volts.in(Volts), volts.in(Volts)),
                log -> {
                    log.motor("TopLeft")
                        .voltage(
                            appliedVoltage.mut_replace(mSwerveMods[0].getDriveVoltage(), Volts))
                        .linearPosition(appliedDistance.mut_replace(mSwerveMods[0].getPosition().distanceMeters, Meters))
                        .linearVelocity(appliedVelocity.mut_replace(mSwerveMods[0].getState().speedMetersPerSecond, MetersPerSecond));
                    log.motor("TopRight")
                        .voltage(
                            appliedVoltage.mut_replace(mSwerveMods[1].getDriveVoltage(), Volts))
                        .linearPosition(appliedDistance.mut_replace(mSwerveMods[1].getPosition().distanceMeters, Meters))
                        .linearVelocity(appliedVelocity.mut_replace(mSwerveMods[1].getState().speedMetersPerSecond, MetersPerSecond));
                    log.motor("BotLeft")
                        .voltage(
                            appliedVoltage.mut_replace(mSwerveMods[2].getDriveVoltage(), Volts))
                        .linearPosition(appliedDistance.mut_replace(mSwerveMods[2].getPosition().distanceMeters, Meters))
                        .linearVelocity(appliedVelocity.mut_replace(mSwerveMods[2].getState().speedMetersPerSecond, MetersPerSecond));
                    log.motor("BotRight")
                        .voltage(
                            appliedVoltage.mut_replace(mSwerveMods[3].getDriveVoltage(), Volts))
                        .linearPosition(appliedDistance.mut_replace(mSwerveMods[3].getPosition().distanceMeters, Meters))
                        .linearVelocity(appliedVelocity.mut_replace(mSwerveMods[3].getState().speedMetersPerSecond, MetersPerSecond));
                },
                this
            ));

    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        ChassisSpeeds speeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(), translation.getY(), rotation, getHeading()) :
            new ChassisSpeeds(translation.getX(),translation.getY(),rotation);
        
        speeds = ChassisSpeeds.discretize(speeds, 0.02);

        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
        
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return mPoseEstimator.getEstimatedPosition();
    }

    public Pose2d getOdomPose(){
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        mPoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public ChassisSpeeds getChassisSpeeds(){
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void setHeading(Rotation2d heading){
        mPoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        mPoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public void driveTest(ChassisSpeeds chassisSpeeds){
        
        dChassisSpeeds = chassisSpeeds;
        if(dChassisSpeeds != null){
            SwerveModuleState[] dStates= Constants.Swerve.swerveKinematics.toSwerveModuleStates(dChassisSpeeds);

            if(dChassisSpeeds.vxMetersPerSecond == 0.0 && dChassisSpeeds.vyMetersPerSecond == 0.0 && dChassisSpeeds.omegaRadiansPerSecond == 0.0) {
                var currentStates = getModuleStates();
                // Keep the wheels at their current angle when stopped, don't snap back to straight
                IntStream.range(0, currentStates.length).forEach(i -> dStates[i].angle = currentStates[i].angle);
            }

            SwerveDriveKinematics.desaturateWheelSpeeds(dStates, Constants.Swerve.maxSpeed);
            setModuleStates(dStates);
        }

        dChassisSpeeds = null;
    }

    public void driveVolts(double left, double right){
        mSwerveMods[0].setVoltage(left);
        mSwerveMods[2].setVoltage(left);
        mSwerveMods[3].setVoltage(right);
        mSwerveMods[1].setVoltage(right);
    }

    public void driveField(ChassisSpeeds speeds){
        speeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] states = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.maxSpeed);
        setModuleStates(states);
    }

    public void xStance(){
        SwerveModuleState frontLeft = new SwerveModuleState(0.0,Rotation2d.fromDegrees(-45));
        SwerveModuleState frontright = new SwerveModuleState(0.0,Rotation2d.fromDegrees(45));
        SwerveModuleState backLeft = new SwerveModuleState(0.0,Rotation2d.fromDegrees(45));
        SwerveModuleState backRight = new SwerveModuleState(0.0,Rotation2d.fromDegrees(-45));

        mSwerveMods[0].setDesiredState(frontLeft, false);
        mSwerveMods[1].setDesiredState(frontright, false);
        mSwerveMods[2].setDesiredState(backLeft,false);
        mSwerveMods[3].setDesiredState(backRight, false);
    }

    public Command runDriveQuasiTest(Direction direction)
    {
        System.out.println("Dynamix Test");
        return sysIdRoutine.quasistatic(direction);
    }

    public Command runDriveDynamTest(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }


    /*Use to update the odometry, adds vision measurement if tags are seen */
    public void updateOdometry(){
        mPoseEstimator.update(getGyroYaw(), getModulePositions());
        swerveOdometry.update(getGyroYaw(), getModulePositions());

        if(getOdomPose().minus(getPose()).getX() > 0.2 || getOdomPose().minus(getPose()).getY() > 0.2){
            swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), getPose());
        }

        for(LimeLightSub limelight : limeLightSubs){
            addVisionMeasurement(limelight);
        }
        
    }

    public void addVisionMeasurement(LimeLightSub limelight){
        LimelightHelpers.PoseEstimate measurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight.getName());

        if(measurement.tagCount >= 1){
            double total = 0;
            for(RawFiducial tag : measurement.rawFiducials){
                total += tag.ambiguity;
            }

            if(total / measurement.rawFiducials.length > 0.2){
                return;
            }

            if(Math.abs(measurement.pose.getRotation().minus(getHeading()).getDegrees()) > 3){
                return;
            }

            if(measurement.pose.getTranslation().getDistance(getPose().getTranslation()) > 1){
                return;
            }

            double kDistanceMod = 0.3;
            double stdDev = kDistanceMod*measurement.avgTagDist / measurement.tagCount;
            mPoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(stdDev,stdDev,999999));
            mPoseEstimator.addVisionMeasurement(
                measurement.pose, 
                measurement.timestampSeconds);

            SmartDashboard.putString(limelight.getName() + ": Vision Measurement", measurement.pose.toString());
        }
    }

    @Override
    public void periodic(){
        updateOdometry();

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }

        SmartDashboard.putNumber("Heading: ", getHeading().getDegrees());
        SmartDashboard.putString("Estimated Pose", getPose().toString());
        SmartDashboard.putString("Odometry Pose", getOdomPose().toString());
    }
}