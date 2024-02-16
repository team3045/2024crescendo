// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PositionerSub extends SubsystemBase {
  private static final TalonFX leftPositioner = new TalonFX(17); //Turns counterclockwise to move armp up
  private static final TalonFX rightPositioner = new TalonFX(16); // turns clockwise to move arm up

  private static final boolean absoluteEncoder = false;

  private static double currAngle;
  private static double desiredAngle;

  public static final double MIN_ANGLE = Units.degreesToRadians(-5); 
  public static final double MAX_ANGLE = Units.degreesToRadians(79); 
  /** Creates a new PositionerSub. */
  public PositionerSub() {
    configMotors();
    /*Resets both values on boot, change if using absolute encoder 
     * if not using absolute it is necessary to zero the arm on boot
    */
    if(!absoluteEncoder){
      leftPositioner.setPosition(0);
      rightPositioner.setPosition(0);
    }
    
    currAngle = Units.rotationsToDegrees(leftPositioner.getPosition().getValue()); //gets position of mechanism in rotations and turns it into degrees
    desiredAngle = currAngle; //sets desired angle to current angle so we dont move
  }

  public void configMotors(){
    leftPositioner.getConfigurator().apply(new TalonFXConfiguration());
    rightPositioner.getConfigurator().apply(new TalonFXConfiguration());

    var configs = new TalonFXConfiguration();

    var slot0Configs = configs.Slot0;
    slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
    slot0Configs.kP = 60;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0.1;
    slot0Configs.kV = 0.12;
    slot0Configs.kS = 0.25; // Approximately 0.25V to get the mechanism moving


    configs.Feedback.SensorToMechanismRatio = (70 / 26) * (5) * (4);

    /*We want to be able to move anywhere within a second 
     * Divide by 60 to go from minute to seconds
    */
    configs.MotionMagic.MotionMagicCruiseVelocity = 5;
    configs.MotionMagic.MotionMagicAcceleration =  10; //Units.radiansPerSecondToRotationsPerMinute(MAX_ANGLE-MIN_ANGLE) / 60;
    configs.MotionMagic.MotionMagicJerk = configs.MotionMagic.MotionMagicAcceleration * 10;

    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftPositioner.getConfigurator().apply(configs);

    var rightConfigs = configs;
    rightConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightPositioner.getConfigurator().apply(rightConfigs);

    //They rotate opposite directions so opposite inverted values
    // leftPositioner.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    // rightPositioner.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
  }

  //angle should be between -5 and 79 degrees or sum like that ask carson
  public void goToAngle(double angle){
    desiredAngle = angle;
    PositionVoltage voltage = new PositionVoltage(0);

    MotionMagicVoltage request = new MotionMagicVoltage(0);
    

    //leftPositioner.setControl(request.withPosition(Units.degreesToRotations(angle)));
    rightPositioner.setControl(request.withPosition(Units.degreesToRotations(angle)));
    //leftPositioner.setControl(voltage.withPosition(angle));
    //rightPositioner.setControl(voltage.withPosition(angle));
    SmartDashboard.putNumber("position output", request.Position);
  }



  @Override
  public void periodic() {
    currAngle = Units.rotationsToDegrees(leftPositioner.getPosition().getValue()); //gets position of mechanism in rotations and turns it into degrees
    SmartDashboard.putNumber("Current Arm Angle", currAngle);
      SmartDashboard.putNumber("Curren rot arm", leftPositioner.getPosition().getValue());
  }
}
