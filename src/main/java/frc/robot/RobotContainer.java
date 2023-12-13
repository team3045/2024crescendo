package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick translationJoystick = new Joystick(0);
    private final Joystick rotationJoystick = new Joystick(1);

    /* Drive Controls */
    /*private final double translationAxis = translationJoystick.getRawAxis(1);
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;*/

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(translationJoystick, 1);
    private final JoystickButton robotCentric = new JoystickButton(translationJoystick, 3);
    private final JoystickButton turnLimelight = new JoystickButton(translationJoystick, PS4Controller.Button.kCircle.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    /*Autonomous Chooser */
    private final Command exampleAutoChoice = new exampleAuto(s_Swerve);
    private final Command pathPlannerAutoChoice = new pathPlannerauto2023lib(s_Swerve);
    private final Command xStanceAuto = new xPosition(s_Swerve);

    SendableChooser<Command> autoChooser = new SendableChooser<>();
    

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -translationJoystick.getRawAxis(1)*0.8,
                () -> -translationJoystick.getRawAxis(0)*0.8, 
                () -> -rotationJoystick.getRawAxis(0)*0.4, 
                () -> robotCentric.getAsBoolean()
            )
        );

        autoChooser.setDefaultOption("Example Auto", exampleAutoChoice);
        autoChooser.addOption("PathPlanner Auto", pathPlannerAutoChoice);
        autoChooser.addOption("X Stance", xStanceAuto);

        SmartDashboard.putData(autoChooser);

        // Configure the button bindings
        configureButtonBindings();
    
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        turnLimelight.whileTrue(new turnToLimelight(s_Swerve, s_Swerve.getYaw().getDegrees()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        //return new exampleAuto(s_Swerve);
        //return new xPosition(s_Swerve);  
        //return new pathPlannerauto2023lib(s_Swerve);  

        return autoChooser.getSelected();
    }
}
