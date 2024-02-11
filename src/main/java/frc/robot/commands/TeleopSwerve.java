package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    private final SlewRateLimiter veloRateLimiter = new SlewRateLimiter(0.1);
    private final SlewRateLimiter rotRateLimiter = new SlewRateLimiter(Math.PI / 4);
    private final SlewRateLimiter translationRateLimiter = new SlewRateLimiter(0.1);

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
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

        //translationVal = filterVelocity(translationVal);
        //strafeVal = filterVelocity(strafeVal);
        // rotationVal = filterRotation(rotationVal);

        Translation2d translation = new Translation2d(translationVal,strafeVal).times(Constants.Swerve.maxSpeed);

        /* Drive */
        s_Swerve.drive(
            translation, 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }

    public double filterVelocity(double input){
        return veloRateLimiter.calculate(input);
    }

    public double filterRotation(double input){
        return rotRateLimiter.calculate(input);
    }

    public Translation2d filterTranslation(Translation2d translation){
        return translation.times(translationRateLimiter.calculate(translation.getNorm()));
    }
}