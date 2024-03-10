// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intaking;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PositionerSub;
import frc.robot.subsystems.ShooterSub;

public class IntakeNote extends Command {

  private Intake intake;
  private ShooterSub shooter;
  private PositionerSub arm;
  public static final TimeOfFlight rangeSensor = new TimeOfFlight(0);
  private static final double MaxDistance = Units.inchesToMeters(4) * 1000; //in mm

  /** Creates a new IntakeNote. */
  public IntakeNote(Intake intake, ShooterSub shooter, PositionerSub arm) {
    this.intake = intake;
    this.shooter = shooter;
    this.arm = arm;

    addRequirements(intake,shooter, arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.intakeEnabled = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.stopShooter();
    arm.goToIntake();

    if(noteDetected()){
      shooter.stopFeed();
      intake.disable();
    }
    else{
      intake.enable();
      shooter.feed();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.disable();
    shooter.stopFeed();

    shooter.runBack();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(noteDetected()){
      return true;
    }
    return false;
  }

  public static boolean noteDetected(){
    double range = rangeSensor.getRange();
    SmartDashboard.putNumber("Range Sensor Value", range);
  
    if(range < MaxDistance){
      return true;
    }
    else
      return false;
    
  }
}
