// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.intaking.IntakeNote;

public class Intake extends SubsystemBase {
  private static final TalonFX intakeMotor = new TalonFX(12, "Canivore 3045");
  private boolean enabled;
  /** Creates a new Intake. */
  public Intake() {
    enabled = false;
  }

  @Override
  public void periodic() {
    if(enabled){
      intakeMotor.set(0.8);
    }
    else
      intakeMotor.set(0);

    SmartDashboard.putBoolean("Note Detected", IntakeNote.noteDetected());
  }

  public void setEnabledOrDisable(){
    enabled = !enabled;
  }

  public void enable(){
    enabled = true;
  }

  public void disable(){
    enabled = false;
  }
  
  public boolean getIntaking(){
    return enabled;
  }
}
