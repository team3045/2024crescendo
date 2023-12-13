package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.graphRobotOdometry;
import frc.robot.commands.xPosition;
import frc.robot.subsystems.Swerve;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class exampleAuto extends SequentialCommandGroup {
    public exampleAuto(Swerve s_Swerve){

        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
             TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(2, 0),
                        new Translation2d(2, 2),
                        new Translation2d(0,2)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(0, 0, new Rotation2d(0)),
                //new Pose2d(0,0, new Rotation2d(0)),
                //new Pose2d(0, 0, new Rotation2d(0)),
            config);

        Trajectory spinSquare = TrajectoryGenerator.generateTrajectory(
                List.of(
                        new Pose2d(0,0,new Rotation2d(0)),
                        new Pose2d(2,0,new Rotation2d(Units.degreesToRadians(90))),
                        new Pose2d(2,2,new Rotation2d(Units.degreesToRadians(180))),
                        new Pose2d(0,2,new Rotation2d(Units.degreesToRadians(270))),
                        new Pose2d(0,0,new Rotation2d(Units.degreesToRadians(360)))
                ),
                config);

        Trajectory spinSquareTwo = TrajectoryGenerator.generateTrajectory(
                List.of(
                        new Pose2d(2,0,new Rotation2d().fromDegrees(90)),
                        new Pose2d(2,2,new Rotation2d(Units.degreesToRadians(180)))
                ),
                config);

        Trajectory spinSquareThree = TrajectoryGenerator.generateTrajectory(
                List.of(
                        new Pose2d(2,2,new Rotation2d(Units.degreesToRadians(180))),
                        new Pose2d(0,2,new Rotation2d(Units.degreesToRadians(-90)))
                ),
                config);

        Trajectory spinSquareFour = TrajectoryGenerator.generateTrajectory(
                List.of(
                        new Pose2d(0,2,new Rotation2d(Units.degreesToRadians(-90))),
                        new Pose2d(0,0,new Rotation2d(Units.degreesToRadians(0)))
                ),
                config);


        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        SwerveControllerCommand spinSquareController =
                new SwerveControllerCommand(
                        spinSquare,
                        s_Swerve::getPose,
                        Constants.Swerve.swerveKinematics,
                        new PIDController(Constants.AutoConstants.kPXController,0,0),
                        new PIDController(Constants.AutoConstants.kPYController,0,0),
                        thetaController,
                        s_Swerve::setAutoModuleStates,
                        s_Swerve
                );

        SwerveControllerCommand spinSquareControllerTwo =
                new SwerveControllerCommand(
                        spinSquareTwo,
                        s_Swerve::getPose,
                        Constants.Swerve.swerveKinematics,
                        new PIDController(Constants.AutoConstants.kPXController,0,0),
                        new PIDController(Constants.AutoConstants.kPYController,0,0),
                        thetaController,
                        s_Swerve::setAutoModuleStates,
                        s_Swerve
                );

        SwerveControllerCommand spinSquareControllerThree =
                new SwerveControllerCommand(
                        spinSquareThree,
                        s_Swerve::getPose,
                        Constants.Swerve.swerveKinematics,
                        new PIDController(Constants.AutoConstants.kPXController,0,0),
                        new PIDController(Constants.AutoConstants.kPYController,0,0),
                        thetaController,
                        s_Swerve::setAutoModuleStates,
                        s_Swerve
                );

        SwerveControllerCommand spinSquareControllerFour =
                new SwerveControllerCommand(
                        spinSquareFour,
                        s_Swerve::getPose,
                        Constants.Swerve.swerveKinematics,
                        new PIDController(Constants.AutoConstants.kPXController,0,0),
                        new PIDController(Constants.AutoConstants.kPYController,0,0),
                        thetaController,
                        s_Swerve::setAutoModuleStates,
                        s_Swerve
                );


        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
            new InstantCommand(() -> s_Swerve.zeroGyro()),
            spinSquareController
            //spinSquareControllerThree,
            //spinSquareControllerFour
            //swerveControllerCommand
            //,new xPosition(s_Swerve)

        );
    }
}