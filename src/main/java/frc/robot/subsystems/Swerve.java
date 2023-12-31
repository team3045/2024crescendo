package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.lang.reflect.Field;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    Field2d robotField2d = new Field2d();

    public Swerve() {
        gyro = new Pigeon2(15,"3045 Canivore");
        gyro.configFactoryDefault();
        zeroGyro();

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

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
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
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public void setAutoModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setAutoDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }


    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
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

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public SwerveModuleState[] getXStance(){
        SwerveModuleState xStates[] = Constants.Swerve.swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0), 
        new Translation2d(0,0));

        xStates[0].angle = new Rotation2d(3*Math.PI / 2 -  Math.atan(Constants.Swerve.trackWidth / Constants.Swerve.wheelBase));
        xStates[1].angle = new Rotation2d(Math.PI / 2 +  Math.atan(Constants.Swerve.trackWidth / Constants.Swerve.wheelBase));
        xStates[2].angle = new Rotation2d(Math.PI / 2 +  Math.atan(Constants.Swerve.trackWidth / Constants.Swerve.wheelBase));
        xStates[3].angle = new Rotation2d(Math.PI / 2 -  Math.atan(Constants.Swerve.trackWidth / Constants.Swerve.wheelBase));

        return xStates;

    }

    public ChassisSpeeds getChassisSpeeds(){
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveAuto(ChassisSpeeds chassisSpeeds){
        SwerveModuleState[] autoSwerveStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(autoSwerveStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods){
            mod.setDesiredState(autoSwerveStates[mod.moduleNumber], false);
        }
    }

    
    //This one should only apply a rotation to chassis speeds
    public void turnToAngle(double omegaRadiansPerSecond){

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, omegaRadiansPerSecond);
        SwerveModuleState[] turnToAngleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(turnToAngleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods){
            mod.setDesiredState(turnToAngleStates[mod.moduleNumber], true);
        }
    }

    //used in turnToLimelight command
    //replaced with turnToAngle for more general use case and hopefully better results
    public void turnToLimelight(double output){
        drive(new Translation2d(0,0),output,false,false);
        SmartDashboard.putNumber("LimelightPID output", output);
        
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());
        robotField2d.setRobotPose(getPose());

        

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);   
            SmartDashboard.putNumber("Mod "+ mod.moduleNumber + "position", Units.metersToInches(mod.getPosition().distanceMeters));
        }

        SmartDashboard.putNumber("Robot Headin: ", getYaw().getDegrees());
        SmartDashboard.putString("Robot Position", getPose().getTranslation().toString());
        SmartDashboard.putData("Robot Position Graph", robotField2d);
        
    }
}