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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.aiming.FullAim;
import frc.robot.commands.intaking.IntakeNote;
import frc.robot.commands.shots.ShootAmp;
import frc.robot.commands.shots.ShootClose;
import frc.robot.commands.shots.ShootMiddleNote;
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
    private final Joystick operator = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = 1;
    private final int strafeAxis = 0;
    private final int rotationAxis = 2;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, PS4Controller.Button.kTriangle.value); //Single Press
    private final JoystickButton ampScore = new JoystickButton(driver, PS4Controller.Button.kCross.value); //Single Press
    private final JoystickButton intakeButton = new JoystickButton(driver, PS4Controller.Button.kR1.value);
    private final JoystickButton fineControl  = new JoystickButton(driver, PS4Controller.Button.kR2.value); //Toggle
    private final JoystickButton shooterModeToggle = new JoystickButton(driver, PS4Controller.Button.kL1.value);
    private final JoystickButton feed = new JoystickButton(driver, PS4Controller.Button.kL2.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, PS4Controller.Button.kL3.value); //Hold down
    
    /*Operator Buttons */
    private final JoystickButton climberUp = new JoystickButton(operator, PS4Controller.Button.kL2.value);
    private final JoystickButton climberDown = new JoystickButton(operator, PS4Controller.Button.kR2.value);
    private final JoystickButton revShooter = new JoystickButton(operator, PS4Controller.Button.kR1.value);
    private final JoystickButton safeShoot = new JoystickButton(operator, PS4Controller.Button.kCircle.value);
    private final JoystickButton pathFindAmp = new JoystickButton(operator, PS4Controller.Button.kSquare.value);
    private final JoystickButton ampMode = new JoystickButton(operator, PS4Controller.Button.kTriangle.value);

    /*SysiId buttons */
    private final JoystickButton quasiForward = new JoystickButton(driver, PS4Controller.Button.kR1.value);
    private final JoystickButton quasiBackward = new JoystickButton(driver, PS4Controller.Button.kR2.value);
    private final JoystickButton dynaForward = new JoystickButton(driver, PS4Controller.Button.kL1.value);
    private final JoystickButton dynaBackward = new JoystickButton(driver, PS4Controller.Button.kL2.value);

    /*Limelights */
    private final LimeLightSub localizer = new LimeLightSub("intake");
    private final LimeLightSub shooterLimelight = new LimeLightSub("shooter");
    private final LimeLightSub frontLimelight = new LimeLightSub("front");

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve(new LimeLightSub[] {localizer,frontLimelight});
    private final ShooterSub shooterSub = new ShooterSub();
    private final PositionerSub positionerSub = new PositionerSub(shooterLimelight);
    private final Intake intake = new Intake();
    private final AutoSub autoSub = new AutoSub(s_Swerve, positionerSub, shooterSub, intake,shooterLimelight);
    private final LEDSub LEDS = new LEDSub();
    private final ElevatorSub elevator = new ElevatorSub();

    /*Commands */
    private final ShootClose ShootClose = new ShootClose(positionerSub, shooterSub);
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
                shooterLimelight
            )
        );

        positionerSub.setDefaultCommand(new FullAim(positionerSub, shooterLimelight, shooterSub, s_Swerve, autoSub));

        localizer.periodic();
        shooterLimelight.periodic();
        positionerSub.periodic();
        intake.periodic();
        elevator.periodic();

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
      
        feed.toggleOnTrue( /*Waits for it to rev up to 29 MPS or 0.5s before feeding */
            new FeedAndShoot(shooterSub, positionerSub).
            raceWith(new SequentialCommandGroup(
                new WaitCommand(0.7), 
                Commands.runOnce(() -> shooterSub.feed()))).
                andThen(new WaitCommand(0.4).andThen(new InstantCommand(() -> shooterSub.stopAll()))));

        // feed.onTrue(new InstantCommand(()-> shooterSub.shootSpeed(60, 60)));

        ampScore.onTrue(shootAmp.andThen(new InstantCommand(() -> TeleopSwerve.disableAmpMode())));

        shooterModeToggle.onTrue(new InstantCommand(() -> TeleopSwerve.toggleShooterMode()));
        

        /*Operator controls */
        climberUp.whileTrue(Commands.runEnd(() -> elevator.goUp(),() -> elevator.stop(),elevator));
        climberDown.whileTrue(Commands.runEnd(() -> elevator.goDown(),() -> elevator.stop(),elevator));
        safeShoot.onTrue(new ShootMiddleNote(shooterSub, positionerSub));
        revShooter.toggleOnTrue(Commands.runEnd(() -> shooterSub.setRev(), ()-> shooterSub.stopShooter(), shooterSub));
        pathFindAmp.whileTrue(autoSub.goToAmp()).
            onFalse(new InstantCommand(() -> LimelightHelpers.setPipelineIndex(shooterLimelight.getName(), 0)));
        ampMode.onTrue(new InstantCommand(() -> TeleopSwerve.toggleAmpMode()).
            alongWith(new InstantCommand(() -> positionerSub.goToAmp()).onlyIf(() -> TeleopSwerve.ampMode == true)));
        

        /*LED triggers */
        new Trigger(() -> true).onTrue(new InstantCommand(() -> LEDS.setDefaultState())); //Base state, stays like this if nothing else is detected
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
        return autoSub.getAutoCommand("Middle 1 2");
    }
}
