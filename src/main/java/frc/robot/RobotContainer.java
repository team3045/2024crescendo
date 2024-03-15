package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.aiming.FullAim;
import frc.robot.commands.intaking.IntakeNote;
import frc.robot.commands.shots.ShootAmp;
import frc.robot.commands.shots.ShootClose;
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
    private final JoystickButton overDrive = new JoystickButton(driver, PS4Controller.Button.kCircle.value); 

    private final JoystickButton zeroGyro = new JoystickButton(driver, PS4Controller.Button.kTriangle.value); //Single Press
    private final JoystickButton fineControl = new JoystickButton(driver, PS4Controller.Button.kR2.value); //Toggle
    private final JoystickButton robotCentric = new JoystickButton(driver, PS4Controller.Button.kL3.value); //Hold down
    private final JoystickButton ampScore = new JoystickButton(driver, PS4Controller.Button.kCross.value); //Single Press
    private final JoystickButton feed = new JoystickButton(driver, PS4Controller.Button.kL2.value);
    private final JoystickButton intakeButton = new JoystickButton(driver, PS4Controller.Button.kR1.value);
    private final JoystickButton safeShoot = new JoystickButton(driver, PS4Controller.Button.kCircle.value);
    private final JoystickButton shooterModeToggle = new JoystickButton(driver, PS4Controller.Button.kL1.value);

    /* Subsystems */
    private final LimeLightSub localizer = new LimeLightSub("limelight");
    private final LimeLightSub shooterLimelight = new LimeLightSub("Some name");
    private final Swerve s_Swerve = new Swerve(localizer);
    private final ShooterSub shooterSub = new ShooterSub();
    private final PositionerSub positionerSub = new PositionerSub(shooterLimelight);
    private final Intake intake = new Intake();
    private final AutoSub autoSub = new AutoSub(s_Swerve, positionerSub, shooterSub, intake,shooterLimelight);
    private final LEDSub LEDS = new LEDSub();

    /*Commands */
    private final ShootMiddleNote ShootClose = new ShootMiddleNote(positionerSub, shooterSub);
    private final IntakeNote intakeNote = new IntakeNote(intake, shooterSub, positionerSub);
    private final ShootAmp shootAmp = new ShootAmp(positionerSub, shooterSub);

    /*Named Commands */

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean(),
                localizer
            )
        );

        positionerSub.setDefaultCommand(new FullAim(positionerSub, localizer, shooterSub, s_Swerve, autoSub));

        //poseEstimation.periodic();
        localizer.periodic();
        positionerSub.periodic();
        intake.periodic();

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
        //intakeButton.onTrue(new IntakeNote(intake, shooterSub, positionerSub));
        intakeButton.toggleOnTrue(intakeNote.alongWith(new InstantCommand(() -> LEDS.setIntaking())));
        fineControl.onTrue(new InstantCommand(() ->
            {if(Constants.Swerve.maxSpeed == 5.0){
                Constants.Swerve.maxSpeed = 2.0;
                Constants.Swerve.maxAngularVelocity = Math.PI / 2;
                Constants.Swerve.normalControl = false;
            }
            else
                Constants.Swerve.maxSpeed = 5.0;
                Constants.Swerve.maxAngularVelocity = Units.degreesToRadians(540);
                Constants.Swerve.normalControl = true;}));
    
        feed.toggleOnTrue(new FeedAndShoot(shooterSub));

        ampScore.onTrue(shootAmp);
        safeShoot.onTrue(new ShootMiddleNote(positionerSub, shooterSub));

        shooterModeToggle.onTrue(new InstantCommand(() -> TeleopSwerve.toggleShooterMode()));

        new Trigger(() -> IntakeNote.noteDetected()).onTrue(new InstantCommand(() -> LEDS.setHasPiece()));
        new Trigger(() -> TeleopSwerve.shooterMode).onTrue(new InstantCommand(() -> LEDS.setTargetting()));
        new Trigger(() -> (TeleopSwerve.shooterMode && shooterLimelight.getTx() < 1 && shooterLimelight.getTy() < 1)).onTrue(new InstantCommand(() -> LEDS.setTargetLock()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoSub.getAutoCommand("3 Middle Right");
    }
}
