// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.AutoSub;
import frc.robot.subsystems.LimeLightSub;

public class TurnToTarget extends Command {
  private LimeLightSub vision;
  private Swerve swerve;
  private AutoSub auto;
  /** Creates a new TurnToTarget. */
  public TurnToTarget(LimeLightSub vision, Swerve swerve, AutoSub auto) {
    this.swerve =swerve;
    this.vision = vision;
    this.auto = auto;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(vision.getTargetSeen()){
      PIDController aController = new PIDController(0.1, 0, 0);
      aController.setSetpoint(0);
      double aOutput = aController.calculate(vision.getTx());

      aController.setTolerance(1);

      System.out.println("turning");

      swerve.driveTest(new ChassisSpeeds(0, 0, aOutput*0.7));

      aController.close();
    }
    else{
      PIDController aController = new PIDController(0.1, 0, 0);
      aController.setSetpoint(auto.getGoalPose(5.0, vision).get().getRotation().getDegrees());
      double aOutput = aController.calculate(swerve.getHeading().getDegrees());

      aController.setTolerance(1);

      System.out.println("turning GYRO");

      swerve.driveTest(new ChassisSpeeds(0, 0, aOutput*0.7));

      aController.close();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
