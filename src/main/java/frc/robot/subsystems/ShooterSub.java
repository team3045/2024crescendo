// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSub extends SubsystemBase {
  private static final TalonFX topMotor = new TalonFX(11);
  private static final TalonFX bottomMotor = new TalonFX(10);
  private static final TalonFX feedMotor = new TalonFX(13);

  private static final double flywheelCircumference = Units.inchesToMeters(4) * Math.PI; //change later

  private enum ShooterSate{
    STOP,
    FEED,
    SHOOTING
  }

  private static ShooterSate state;
  
  /** Creates a new ShooterSub. */
  public ShooterSub() {
    configureMotors();
  }

  public void configureMotors(){
    topMotor.getConfigurator().apply(new TalonFXConfiguration());
    bottomMotor.getConfigurator().apply(new TalonFXConfiguration());
    feedMotor.getConfigurator().apply(new TalonFXConfiguration());

    // in init function
    var talonFXConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // no output for error derivative

    slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
    talonFXConfigs.Feedback.SensorToMechanismRatio = 1; //currently arbitary value

    // set Motion Magic Velocity settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

    topMotor.getConfigurator().apply(talonFXConfigs);
    bottomMotor.getConfigurator().apply(talonFXConfigs);

    state = ShooterSate.STOP;
  }

  //speeds in rotations per second
  public void shootSpeed(double topSpeed, double bottomSpeed){
    //Velocity the motors should go to rot per sec
    var request = new MotionMagicVelocityVoltage(0);

    //sets it to go at 80 rot per sec with default acceleration
    topMotor.setControl(request.withVelocity(topSpeed));
    bottomMotor.setControl(request.withVelocity(bottomSpeed));

    state = ShooterSate.SHOOTING;
  }

  public void feed(){
    state = ShooterSate.FEED;
    feedMotor.set(0.15);
  }

  public void stopShooter(){
    shootSpeed(0, 0);
  }

  public void stopFeed(){
    feedMotor.set(0);
  }

  /*Use A regression calculated from testing to determine flyhweel speed in meter/s based on distance */
  //This mps is the speed of the flywheels not of the ball on exit
  //ball speed on exit would be the tangential speed which should be half not accounting for slip or friction
  public static double calculateMPS(double distance){
    double MPS = Math.pow(distance, 3) + 2*Math.pow(distance, 2) + distance + 0; //Regression
    return MPS;
  }

  //MPS = RPS * circumference(meters)
  public static double MPSToRPS(double wheelMPS, double circumference){
    double wheelRPS = wheelMPS / circumference;
    return wheelRPS;
  }

  public void shootDistance(double distance){
    double RPS = MPSToRPS(calculateMPS(distance), flywheelCircumference);
    this.shootSpeed(RPS,RPS);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}