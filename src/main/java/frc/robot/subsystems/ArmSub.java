// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSub extends SubsystemBase {
  private static TalonFX armMotor = new TalonFX(14);
  private static TalonFX followerMotor = new TalonFX(15);
  private static Encoder armEncoder = new Encoder(null, null);

  private static double armEncoderOffset = 0.1; //SET LATER, in rotations
  /** Creates a new ArmSub. */
  public ArmSub() {
    configureMotors();
  }

  public void configureMotors(){
    armMotor.getConfigurator().apply(new TalonFXConfiguration());
    followerMotor.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration armConfigs = new TalonFXConfiguration();
    armConfigs.Feedback.FeedbackRotorOffset = armEncoderOffset;
    armConfigs.Feedback.SensorToMechanismRatio = 1; //set sensor to mechanism ratio
    
    var slot0Configs = armConfigs.Slot0;
    slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // no output for error derivative

    var motionMagicConfigs = armConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)   

    armMotor.getConfigurator().apply(armConfigs);
    followerMotor.getConfigurator().apply(armConfigs);

    armMotor.setNeutralMode(NeutralModeValue.Brake);
    followerMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
