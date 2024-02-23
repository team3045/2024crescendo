// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PositionerSub;
import frc.robot.subsystems.ShooterSub;

public class IntakeNote extends Command {

  //Change to match port
  private static final I2C.Port i2cPort = I2C.Port.kOnboard;
  private Intake intake;
  private ShooterSub shooter;
  private PositionerSub arm;
  private static final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

   /**
   * A Rev Color Match object is used to register and detect known colors. This can 
   * be calibrated ahead of time or during operation.
   * 
   * This object uses a simple euclidian distance to estimate the closest match
   * with given confidence range.
   */
  private static final ColorMatch colorMatch = new ColorMatch();
  /*Colors */
  private static final Color noteColor = new Color("#7B6320");

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
    colorMatch.addColorMatch(noteColor);
    colorMatch.setConfidenceThreshold(0.90);
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

  public boolean noteDetected(){
    Color detectedColor = colorSensor.getColor();
    SmartDashboard.putString("Color Sensor", detectedColor.toString());

    /*Matches detectedColor to NoteColor 
     * if confidence is below 0.9 it returns null, otherwise it returns detectedColor
    */
    ColorMatchResult result = colorMatch.matchColor(detectedColor); 
  
    if(result == null){
      return false;
    }
    else
      return true;
    
  }
}
