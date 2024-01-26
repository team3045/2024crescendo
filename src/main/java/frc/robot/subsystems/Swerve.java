package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.SwerveModule;
import frc.robot.Constants.EstimationConstants;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.stream.IntStream;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry odometry;
    public SwerveDrivePoseEstimator mPoseEstimator;
    public LimeLightSub limeLightSub;
    public SwerveModule[] mSwerveMods;
    public static Pigeon2 gyro;
    private ChassisSpeeds dChassisSpeeds;
    Field2d robotField2d = new Field2d();
    /**
    * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
    * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
    */
    //ie how much it trusts its current estimate
    private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(3));

    /**
    * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
    * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
    */
    //how much it trusts the vision measurements
     private static Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.9, 0.9, Units.degreesToRadians(30));


    public Swerve(LimeLightSub limeLightSub) {
        this.limeLightSub = limeLightSub;
        gyro = new Pigeon2(15,"3045 Canivore");
        //apply default config
        gyro.configFactoryDefault();

        //zeroGyro();
        gyro.setYaw(Units.degreesToRadians(EstimationConstants.robotStartPose.getRotation().getZ()));

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        mPoseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
             getYaw(), 
             getModulePositions(), 
             EstimationConstants.robotStartPose.toPose2d(),
             stateStdDevs,
             visionMeasurementStdDevs);

        odometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions(),EstimationConstants.robotStartPose.toPose2d());

        //odometry.resetPosition(getYaw(), getModulePositions(), getPose());
        //mPoseEstimator.resetPosition(getYaw(), getModulePositions(), getPose());

    
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    //Specificalled 
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    //returns estimated position including vision and odometry
    public Pose2d getPose() {
        return mPoseEstimator.getEstimatedPosition();
    }

    //returns position from odometry only
    public Pose2d getOdomPose2d(){
        return odometry.getPoseMeters();
    }

    //pass in current position to reset where you are to be origin
    public void resetPose(Pose2d pose) {
        mPoseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
        odometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    //Getters for swerve modules
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

    //sets your current heading to be zero
    public void zeroGyro(){
        gyro.setYaw(0);
    }

    //returns yaw according to gyro NOT poseestimator 
    //TODO: change to pose estimator yaw, and make std dev for vision estimate angles very large
    //aka we only trust the gyro for angle estimates
    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    //
    public ChassisSpeeds getChassisSpeeds(){
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
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

    //adds the vision measurement to pose estimator if target is seen and the id is correct
    //scales the std dev based on distance from tag TLDR: if we're farther away we trust vision less
    public void addVision(){
        if (limeLightSub.getTargetSeen()&&EstimationConstants.idPoses.containsKey(limeLightSub.getID())){
            double xDistance = limeLightSub.getCamToTargetTransform().getTranslation().getX();
            double yDistance = limeLightSub.getCamToTargetTransform().getTranslation().getZ();
        
            //if were farther than 3 meters away than we discard the measurment by making the std dev huge
            double kDistanceMod = limeLightSub.getNorm() > 3.0 ? 10000 : 0.3; 
        

            visionMeasurementStdDevs = VecBuilder.fill(0.3*xDistance*kDistanceMod, 0.3*kDistanceMod*yDistance, Units.degreesToRadians(30));
            mPoseEstimator.setVisionMeasurementStdDevs(visionMeasurementStdDevs);
            mPoseEstimator.resetPosition(getYaw(), getModulePositions(), limeLightSub.getVisionMeasurement());

            odometry.resetPosition(getYaw(), getModulePositions(), limeLightSub.getVisionMeasurement());

            mPoseEstimator.addVisionMeasurement(
                limeLightSub.getVisionMeasurement(),
                limeLightSub.getTimeStampSeconds());
        }
        else{
            mPoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.3,0.3,Units.degreesToRadians(3)));
            mPoseEstimator.addVisionMeasurement(getOdomPose2d(), System.currentTimeMillis()-Constants.startTime);

        }
    }

    @Override
    public void periodic(){
        mPoseEstimator.updateWithTime(System.currentTimeMillis()-Constants.startTime, getYaw(), getModulePositions());
        //addVision();

        robotField2d.setRobotPose(getPose());

        odometry.update(getYaw(), getModulePositions());

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }

        SmartDashboard.putNumber("Robot Headin: ", getYaw().getDegrees());
        SmartDashboard.putString("Robot Position", getPose().getTranslation().toString());
        SmartDashboard.putData("Robot Position Graph", robotField2d);

        SmartDashboard.putString("Odometry", getOdomPose2d().toString());
        SmartDashboard.putString("start pos", EstimationConstants.robotStartPose.toPose2d().toString());
        
    }
}