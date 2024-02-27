package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import frc.robot.Constants.EstimationConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
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
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveDrivePoseEstimator mPoseEstimator;
    public LimeLightSub limeLightSub;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    private ChassisSpeeds dChassisSpeeds;

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
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    Constants.Swerve.normalControl ? translation.getX(): translation.getX(), 
                                    Constants.Swerve.normalControl ? translation.getY(): translation.getY(), 
                                    rotation, 
                                    getHeading()
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
        return swerveOdometry.getPoseMeters();
    }

    public Pose2d getOdomPose(){
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public ChassisSpeeds getChassisSpeeds(){
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        System.out.println("zero heading");
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
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

    public void driveField(ChassisSpeeds speeds){
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds,getHeading());
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
            mPoseEstimator.resetPosition(getHeading(), getModulePositions(), new Pose2d(limeLightSub.getVisionMeasurement().getTranslation(),getHeading()));

            swerveOdometry.resetPosition(getHeading(), getModulePositions(), new Pose2d(limeLightSub.getVisionMeasurement().getTranslation(),getHeading()));

            /*mPoseEstimator.addVisionMeasurement(
                new Pose2d(limeLightSub.getVisionMeasurement().getTranslation(),getYaw()),
                limeLightSub.getTimeStampSeconds());*/
        }
        else{
            mPoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.3,0.3,Units.degreesToRadians(3)));
            mPoseEstimator.addVisionMeasurement(getOdomPose(), System.currentTimeMillis()-Constants.startTime);

        }
    }

    @Override
    public void periodic(){
        mPoseEstimator.updateWithTime(System.currentTimeMillis()-Constants.startTime, getGyroYaw(), getModulePositions());
        //addVision(); //disabled while testing odom, causes issues by resetting odometry
        swerveOdometry.update(getGyroYaw(), getModulePositions());

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }

        SmartDashboard.putNumber("Heading: ", getHeading().getDegrees());
    }
}