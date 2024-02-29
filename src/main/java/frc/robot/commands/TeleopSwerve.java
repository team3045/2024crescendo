package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.LimeLightSub;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private LimeLightSub vision;
    public static boolean shooterMode = false;

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

        rotationVal = shooterMode ? calcRotationShooterMode() : rotationVal;
        System.out.println(shooterMode + " AND " + rotationVal);

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
    }

    public double calcRotationShooterMode(){
        vision.setAimingPipeline();
        if(vision.getTargetSeen()){
            PIDController aController = new PIDController(0.03, 0, 0);
            aController.setSetpoint(0);
            double aOutput = aController.calculate(vision.getTx());

            aController.setTolerance(1);


            aController.close();
            vision.setLocalizerPipeline();
            return aOutput;
        }

        vision.setLocalizerPipeline();
        return 0;
    }
}