package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = 1;
    private final int strafeAxis = 0;
    private final int rotationAxis = 2;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, PS4Controller.Button.kTriangle.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, PS4Controller.Button.kR2.value);
    private final JoystickButton intakeButton = new JoystickButton(driver, PS4Controller.Button.kR1.value);
    private final JoystickButton fineControl = new JoystickButton(driver, PS4Controller.Button.kCross.value);
    private final JoystickButton overDrive = new JoystickButton(driver, PS4Controller.Button.kCircle.value); 
    private final JoystickButton shooterTest = new JoystickButton(driver, PS4Controller.Button.kL1.value);
    private final JoystickButton feedTest = new JoystickButton(driver, PS4Controller.Button.kR1.value);
    private final JoystickButton pathPlannerFindPose = new JoystickButton(driver, PS4Controller.Button.kSquare.value);
    private final JoystickButton positionButton = new JoystickButton(driver, PS4Controller.Button.kL2.value);


    /* Subsystems */
    private final LimeLightSub localizer = new LimeLightSub("limelight");
    private final LimeLightSub shooterLimelight = new LimeLightSub("Some name");
    private final Swerve s_Swerve = new Swerve();
    private final AutoSub autoSub = new AutoSub(s_Swerve);
    private final ShooterSub shooterSub = new ShooterSub();
    private final PositionerSub positionerSub = new PositionerSub(shooterLimelight);
    //private final PoseEstimationPhoton poseEstimation = new PoseEstimationPhoton(new PhotonCamera("limelight"), s_Swerve);

    private final Intake intake = new Intake();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        //poseEstimation.periodic();
        localizer.periodic();
        positionerSub.periodic();

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
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        intakeButton.onTrue(new InstantCommand(() -> intake.setEnabledOrDisable()));

        fineControl.onTrue(new InstantCommand(() ->
            {if(Constants.Swerve.maxSpeed == 4.5){
                Constants.Swerve.maxSpeed = 2.0;
                Constants.Swerve.maxAngularVelocity = Math.PI / 2;
                Constants.Swerve.normalControl = false;
            }
            else
                Constants.Swerve.maxSpeed = 4.5;
                Constants.Swerve.maxAngularVelocity = Math.PI * 2;
                Constants.Swerve.normalControl = true;}));

        overDrive.onTrue(new InstantCommand(() ->
            {if(Constants.Swerve.maxSpeed == 6.0){
                Constants.Swerve.maxSpeed = 4.5;
                Constants.Swerve.maxAngularVelocity = Math.PI * 2;
                Constants.Swerve.normalControl = true;
            }
            else
                Constants.Swerve.maxSpeed = 6.0;
                Constants.Swerve.maxAngularVelocity = Math.PI * 2;
                Constants.Swerve.normalControl = false;}));
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        //goes to goalPose if target is in sight otherwise just stays in place
        //possible errpr with sequential command group index out of bounds
        pathPlannerFindPose.whileTrue( new SequentialCommandGroup(
            //autoSub.getOnFlyCommand(autoSub.getGoalPose(5.0, vision).orElse(s_Swerve.getPose())),
            autoSub.getOnFlyCommand(autoSub.getGoalPose(5.0, localizer).get()))
            //new TurnToTarget(vision, s_Swerve, autoSub))
        );

        shooterTest.whileTrue(new InstantCommand(() -> shooterSub.shootSpeed(-80, -80)));
        shooterTest.whileFalse(new InstantCommand(() -> shooterSub.stopShooter()));

        feedTest.whileTrue(new InstantCommand(() -> shooterSub.feed()));
        feedTest.whileFalse(new InstantCommand(() -> shooterSub.stopFeed()));

        positionButton.onTrue(new InstantCommand(() -> positionerSub.goToAngle(36.5)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}
