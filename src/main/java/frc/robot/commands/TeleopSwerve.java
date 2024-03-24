package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.LimeLightSub;
import frc.robot.subsystems.Swerve;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private LimeLightSub vision;
    public static boolean shooterMode = false;
    public static boolean ampMode = false;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, LimeLightSub vision) {
        this.s_Swerve = s_Swerve;
        this.vision = vision;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        rotationVal = shooterMode ? calcRotationShooterMode(rotationVal) : rotationVal;
        rotationVal = ampMode ? turnToAngle(-90) : rotationVal;

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }

    public static void toggleShooterMode(){
        System.out.println("toggle shooter" + !shooterMode);
        shooterMode = !shooterMode;
        ampMode = false;
    }

    public static void toggleAmpMode(){
        ampMode = !ampMode;
        shooterMode = false;
    }

    public static void disableAmpMode(){
        ampMode = false;
    }

    public double calcRotationShooterMode(double rotation){
        vision.setAimingPipeline();
        if(vision.getTargetSeen()){
            PIDController aController = new PIDController(0.01, 0, 0);
            aController.setSetpoint(getSideSetpoint());
            double aOutput = aController.calculate(vision.getTx());

            aController.setTolerance(0.5);


            aController.close();
            return aOutput;
        }
        // else{
        //     return turnToAngle(getRotationGoal());
        // }
        return rotation;
    }

    public double turnToAngle(double goal){
        PIDController aController = new PIDController(0.01, 0, 0);
        aController.setSetpoint(goal);
        double aOutput = aController.calculate(s_Swerve.getHeading().getDegrees());

        aController.setTolerance(0.5);

        aController.close();
        return aOutput;
    }
    
    public double getRotationGoal(){
        Optional<Alliance> ally = DriverStation.getAlliance();
        if(ally.isPresent()){
            Pose3d tagPose;

            if(ally.get() == Alliance.Red)
                tagPose = Constants.EstimationConstants.fieldLayout.getTagPose(3).orElse(new Pose3d(s_Swerve.getPose()));
            else
                tagPose = Constants.EstimationConstants.fieldLayout.getTagPose(5).orElse(new Pose3d(s_Swerve.getPose()));

            Rotation2d goalRotation = tagPose.getRotation().toRotation2d().rotateBy(s_Swerve.getHeading());
            return goalRotation.getDegrees();
        }
        else{
            System.out.println("DriverStation No Color");
            return s_Swerve.getHeading().getDegrees();
        }
  }

  public double getSideSetpoint(){
    double sideKP = -1;
    if(Math.abs(vision.getHorizontalDistanceToSpeaker()) > 3.5){
        System.out.println(0 + vision.getHorizontalDistanceToSpeaker() * sideKP);
        return 0 + vision.getHorizontalDistanceToSpeaker() * sideKP;
    }
    else
        return 0;
  }
}