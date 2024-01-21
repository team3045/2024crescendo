package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
    private final Joystick driveController = new Joystick(0);
    //private final Joystick rotationJoystick = new Joystick(1);    

    /* Drive Controls */
    private final int translationAxis = PS4Controller.Axis.kLeftY.value;
    private final int strafeAxis = PS4Controller.Axis.kLeftX.value;
    private final int rotationAxis = PS4Controller.Axis.kRightX.value;


    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driveController, PS4Controller.Button.kTriangle.value);
    private final JoystickButton robotCentric = new JoystickButton(driveController, PS4Controller.Button.kR2.value);

    private final JoystickButton fieldSwerveCommand = new JoystickButton(driveController, PS4Controller.Button.kCircle.value);
    private final JoystickButton driveToTarget = new JoystickButton(driveController, PS4Controller.Button.kCross.value);

    

    


    /* Subsystems */
    private final LimeLightSub vision = new LimeLightSub();
    private final Swerve s_Swerve = new Swerve(vision);
    //private final PoseEstimationPhoton poseEstimation = new PoseEstimationPhoton(new PhotonCamera("limelight"), s_Swerve);
    

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve,
                () -> -1 * driveController.getRawAxis(translationAxis)*0.8,
                () -> -1 * driveController.getRawAxis(strafeAxis)*0.8, 
                () -> -driveController.getRawAxis(rotationAxis)*0.4 ,
                () -> robotCentric.getAsBoolean()
            )
        );

        //poseEstimation.periodic();
        vision.periodic();

        

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
        driveToTarget.whileTrue(new DriveToTarget(5.0, vision, s_Swerve));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new InstantCommand() {};
    }
}
