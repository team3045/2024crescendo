package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class xPosition extends CommandBase {
    private Swerve s_Swerve;
    private SwerveModule[] mSwerveMods;


    public xPosition(Swerve s_Swerve){
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
    }

    public void setXPosition(){
        SwerveModuleState topRight = new SwerveModuleState(0, Constants.Swerve.Mod1.angleOffset.getDegrees() < 315 ? new Rotation2d(Constants.Swerve.Mod1.angleOffset.getDegrees()+45) : new Rotation2d(Constants.Swerve.Mod1.angleOffset.getDegrees()+45-360));
        SwerveModuleState topLeft = new SwerveModuleState(0, Constants.Swerve.Mod0.angleOffset.getDegrees() < 315 ? new Rotation2d(Constants.Swerve.Mod1.angleOffset.getDegrees()+45) : new Rotation2d(Constants.Swerve.Mod1.angleOffset.getDegrees()+45-360));
        SwerveModuleState botLeft = new SwerveModuleState(0, Constants.Swerve.Mod2.angleOffset.getDegrees() < 315 ? new Rotation2d(Constants.Swerve.Mod1.angleOffset.getDegrees()+45) : new Rotation2d(Constants.Swerve.Mod1.angleOffset.getDegrees()+45-360));
        SwerveModuleState botRight = new SwerveModuleState(0, Constants.Swerve.Mod3.angleOffset.getDegrees() < 315 ? new Rotation2d(Constants.Swerve.Mod1.angleOffset.getDegrees()+45) : new Rotation2d(Constants.Swerve.Mod1.angleOffset.getDegrees()+45-360));

        mSwerveMods[0].setDesiredState(topLeft, false);
        mSwerveMods[1].setDesiredState(topRight, false);
        mSwerveMods[2].setDesiredState(botLeft, false);
        mSwerveMods[3].setDesiredState(botRight, false);
    }
    
    @Override
    public void execute() {
       setXPosition();
    }

   }

