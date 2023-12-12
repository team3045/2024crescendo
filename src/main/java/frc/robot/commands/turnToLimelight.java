// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.limelightVision;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class turnToLimelight extends PIDCommand {
  /** Creates a new turnToLimelight. */
  public turnToLimelight(Swerve s_Swerve) {
    super(
        // The controller that the command will use
        new PIDController(Constants.kPXAngleOffset, 0, 0),
        // This should return the measurement
        () -> s_Swerve.getYaw().getDegrees(),
        // This should return the setpoint (can also be a constant)
        () -> limelightVision.getTX(),
        // This uses the output
        output -> {
          // Use the output here
          s_Swerve.turnToLimelight(output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.

    addRequirements(s_Swerve);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
