// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.limelightVision;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class driveToLimelightX extends PIDCommand {
  /** Creates a new driveToLimelightX. */
  //pass in original robot heading as angleoffset to replicate robot centric turning
  public driveToLimelightX(Swerve s_Swerve) {
    super(
        // The controller that the command will use
        new PIDController(Constants.kPXGain,0, 0),
        // This should return the measurement
        () -> limelightVision.getDistanceX(),
        // This should return the setpoint (can also be a constant)
        () -> Constants.distanceDesired,
        // This uses the output
        output -> {
          output = Math.abs(output) < 0.1 ? 0 : output;
          s_Swerve.driveAuto(new ChassisSpeeds(-output, 0, 0));
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
