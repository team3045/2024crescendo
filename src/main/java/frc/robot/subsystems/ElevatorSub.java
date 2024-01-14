// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSub extends SubsystemBase {

  public static enum ElevatorState{
    AMP,
    SOURCE,
    SHOOT,
    INTAKE;
  }

   public static ElevatorState state;

  //replace with whatever vision system is used
  private LimelightSub visionSub;

  private TalonFX rightElevator; 
  private TalonFX leftElevator;
  private TalonFX wristMotor;

  public static double elevatorPosMeters;
  public static double elevatorPosEncoderTicks;
  public static double elevatorMaxMeters = Units.feetToMeters(4);
  public static double elevatorMaxEncoderTicks;

  public static final double kEncoderTickToMeter = 1.0 / 2048.0 * 0.1 * Math.PI;
  public static final double wristGearRatio = 1/100;
  public static final double elevatorGearRatio = 1/7.75;

  public static final ShuffleboardTab tab = Shuffleboard.getTab("Elevator");

  public static final Map<ElevatorState, Double> stateToHeight = 
    Map.of(
      ElevatorState.AMP, Units.feetToMeters(3),
      ElevatorState.SOURCE, Units.feetToMeters(2),
      ElevatorState.SHOOT, Units.feetToMeters(1),
      ElevatorState.INTAKE, Units.feetToMeters(0));

  /** Creates a new ElevatorSub. */
  public ElevatorSub(LimelightSub vision) {
    state = ElevatorState.INTAKE;

    rightElevator = new TalonFX(0);
    leftElevator = new TalonFX(1);

    rightElevator.configFactoryDefault();
    leftElevator.configFactoryDefault();
    leftElevator.follow(rightElevator);

    rightElevator.setSelectedSensorPosition(0);
    leftElevator.setSelectedSensorPosition(0);

    wristMotor = new TalonFX(2);
    
    elevatorPosEncoderTicks = rightElevator.getSelectedSensorPosition();
    elevatorPosMeters = getEncoderMeters();

    elevatorMaxEncoderTicks = elevatorMaxMeters / kEncoderTickToMeter / elevatorGearRatio;

  }

  public double getEncoderMeters(){
    return rightElevator.getSelectedSensorPosition() * kEncoderTickToMeter * elevatorGearRatio;
  }

  public void setHeight(double desHeightMeters){

    if(desHeightMeters > stateToHeight.get(ElevatorState.AMP)) {
      desHeightMeters = stateToHeight.get(ElevatorState.AMP); 
      state = ElevatorState.AMP;
    }
    else if (desHeightMeters > stateToHeight.get(ElevatorState.SOURCE)) 
        state = ElevatorState.AMP;  
    else if (desHeightMeters > stateToHeight.get(ElevatorState.SHOOT))
      state = ElevatorState.SOURCE;
    else if (desHeightMeters > stateToHeight.get(ElevatorState.SHOOT) / 2)
      state = ElevatorState.SHOOT;
    else 
      state = ElevatorState.INTAKE;

    double desHeightEncoderTicks = desHeightMeters/elevatorMaxMeters * elevatorPosEncoderTicks;
    double diff = desHeightEncoderTicks - rightElevator.getSelectedSensorPosition();

    rightElevator.set(ControlMode.Position, diff);
    leftElevator.follow(rightElevator);
  }

  
  public void setSpeed(double speed){
    rightElevator.set(ControlMode.PercentOutput, speed);
    leftElevator.follow(rightElevator);
  }

  public ElevatorState getState(){

    if(getEncoderMeters() > stateToHeight.get(ElevatorState.AMP)) {
      setHeight(stateToHeight.get(ElevatorState.AMP));
      state = ElevatorState.AMP;
    }
    else if (getEncoderMeters() > stateToHeight.get(ElevatorState.SOURCE)) 
        state = ElevatorState.AMP;
    else if (getEncoderMeters() > stateToHeight.get(ElevatorState.SHOOT))
      state = ElevatorState.SOURCE;
    else if (getEncoderMeters() > stateToHeight.get(ElevatorState.SHOOT) / 2)
      state = ElevatorState.SHOOT;
    else 
      state = ElevatorState.INTAKE;

    return state;
  }

  public void setAmp(){
    state = ElevatorState.AMP;
    setHeight(stateToHeight.get(state));
  }

  public void setSource(){
    state = ElevatorState.SOURCE;
    setHeight(stateToHeight.get(state));
  }

  public void setSpeaker(){
    state = ElevatorState.SHOOT;
    setHeight(stateToHeight.get(state));
  }

  public void setIntake(){
    state = ElevatorState.INTAKE;
    setHeight(stateToHeight.get(state));
  }


  @Override
  public void periodic() {
    tab.add("Elevator Meters", getEncoderMeters());
    tab.add("Elevator Encoder Ticks", rightElevator.getSelectedSensorPosition());
    tab.add("Elevator State", getState());
  }
}
