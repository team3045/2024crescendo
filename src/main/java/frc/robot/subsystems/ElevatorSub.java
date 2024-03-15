// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSub extends SubsystemBase {
  private static TalonFX armMotor = new TalonFX(22, "Canivore 3045"); //right
  private static TalonFX followerMotor = new TalonFX(21, "Canivore 3045"); //left

  private static double armEncoderOffset = 0.1; //SET LATER, in rotations
  /** Creates a new ArmSub. */
  public ElevatorSub() {
    configureMotors();
    SmartDashboard.putNumber("Desired Elevator Pos", 0);
  }

  public void configureMotors(){
    armMotor.getConfigurator().apply(new TalonFXConfiguration());
    followerMotor.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration armConfigs = new TalonFXConfiguration();
    armConfigs.Feedback.SensorToMechanismRatio = 1; //set sensor to mechanism ratio
    
    var slot0Configs = armConfigs.Slot0;
    slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 60; 
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // no output for error derivative

    var motionMagicConfigs = armConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)   

    armMotor.getConfigurator().apply(armConfigs);
    followerMotor.getConfigurator().apply(armConfigs);

    armMotor.setNeutralMode(NeutralModeValue.Brake);
    followerMotor.setNeutralMode(NeutralModeValue.Brake);

    armMotor.setPosition(0);
    followerMotor.setPosition(0);
  }

  @Override
  public void periodic() {
    //goToPos(SmartDashboard.getNumber("Desired Elevator Pos", 0));
  }

  public void goToPos(double desiredPos){
    var request = new MotionMagicVoltage(0);
    armMotor.setControl(request.withPosition(desiredPos));
    System.out.println(desiredPos);
    followerMotor.setControl(new Follower(21, false));

  }

  public void goUp(){
    armMotor.set(0.5);
    followerMotor.set(0.5);
  }

  public void goDown(){
    armMotor.set(-0.5);
    followerMotor.set(-0.5);
  }

  public void stop(){
    armMotor.set(0);
    followerMotor.set(0);
  }
}
