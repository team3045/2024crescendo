// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ElevatorConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ElevatorMovementCommand extends PIDCommand {
  private static TalonFX leftArm = new TalonFX(ElevatorConstants.leftArmID);
  private static TalonFX rightArm = new TalonFX(ElevatorConstants.rightArmID);
  /** Creates a new ElevatorMovementCommand. */
  public ElevatorMovementCommand(StatusSignal<Double> pos) {
    super(
        // The controller that the command will use
        new PIDController(ElevatorConstants.kElevatorP, 0, 0),
        // This should return the measurement
        () -> getMeasurement(rightArm),
        // This should return the setpoint (can also be a constant)
        () -> pos.getValueAsDouble(),
        // This uses the output
        output -> {
          rightArm.setControl
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }


  public static double getMeasurement(TalonFX talonFX){
    return talonFX.getPosition().getValueAsDouble();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
