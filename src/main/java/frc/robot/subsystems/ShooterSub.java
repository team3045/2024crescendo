// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSub extends SubsystemBase {
  private static final TalonFX topMotor = new TalonFX(14);
  private static final TalonFX bottomMotor = new TalonFX(15);
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
    slot0Configs.kP = 10; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // no output for error derivative

    slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
    talonFXConfigs.Feedback.SensorToMechanismRatio = 1.5; //currently arbitary value

    // set Motion Magic Velocity settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 400;
    motionMagicConfigs.MotionMagicAcceleration = 4000; // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigs.MotionMagicJerk = 40000; // Target jerk of 4000 rps/s/s (0.1 seconds)

    topMotor.getConfigurator().apply(talonFXConfigs);
    bottomMotor.getConfigurator().apply(talonFXConfigs);
    feedMotor.getConfigurator().apply(talonFXConfigs);
    topMotor.setPosition(0);
    bottomMotor.setPosition(0);

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

  public void stopAll(){
    stopFeed();
    stopShooter();
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

  public static double RPSToMPS(double wheelRPS, double circumference){
    double wheelMPS = wheelRPS*circumference;
    return wheelMPS;
  }

  public void shootDistance(double distance){
    double RPS = MPSToRPS(calculateMPS(distance), flywheelCircumference);
    this.shootSpeed(RPS,RPS);
  }

  public void shootPct(){
    topMotor.set(-0.9);
    bottomMotor.set(-0.95);
  }

  public void shootAmp(){
    topMotor.set(-0.12);
    bottomMotor.set(-0.19);
  }

  public double getCurrentSpeedMPS(){
    return RPSToMPS(bottomMotor.getVelocity().getValueAsDouble(),flywheelCircumference);
  }

  public void setRev(){
    topMotor.set(-0.70);
    bottomMotor.set(-0.70);
  }

  /*runs it back a little bit for intaking */
  public void runBack(){
    var request = new MotionMagicVoltage(0);
    Slot1Configs temp = new Slot1Configs();
    temp.kS = 0.25; // Add 0.25 V output to overcome static friction
    temp.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    temp.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    temp.kP = 2.5; // An error of 1 rps results in 0.11 V output
    temp.kI = 0; // no output for integrated error
    temp.kD = 0; // no output for error derivative
    feedMotor.getConfigurator().apply(temp);
    feedMotor.setPosition(0);
    feedMotor.setControl(request.withPosition(-0.5).withSlot(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Current Speed", getCurrentSpeedMPS());
  }
}
