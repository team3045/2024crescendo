// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PositionerSub extends SubsystemBase {
  private DutyCycleEncoder absEncoder = new DutyCycleEncoder(1); //Encoder on channel 1
  private final double absEncoderOffset = 0; //range 0-1, in rotations I presume
 
  private static final TalonFX leftPositioner = new TalonFX(17); //Turns counterclockwise to move armp up
  private static final TalonFX rightPositioner = new TalonFX(16); // turns clockwise to move arm up

  private LimeLightSub vision;

  private static final boolean absoluteEncoder = false;

  private static double currAngle;
  private static double desiredAngle;

  public static final double MIN_ANGLE = Units.degreesToRadians(-5); 
  public static final double MAX_ANGLE = Units.degreesToRadians(70); 
  /** Creates a new PositionerSub. */
  public PositionerSub(LimeLightSub vision) {
    /*Initialize our limelight for the shooter 
     * Seperate from our limelight for localization
    */
    this.vision = vision;

    /*Config both positioners */
    configMotors();

    /*Resets both values on boot, change if using absolute encoder 
     * if not using absolute it is necessary to zero the arm on boot
    */
    if(!absoluteEncoder){
      leftPositioner.setPosition(0);
      rightPositioner.setPosition(0);
    }
    else{
      absEncoder.setPositionOffset(absEncoderOffset);
      leftPositioner.setPosition(absEncoder.getAbsolutePosition());
      rightPositioner.setPosition(absEncoder.getAbsolutePosition());
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
    /* Rps at cruise velocity, should be in rotations of mechanism rather than just motor axle
    * Accel: Takes 0.5s to reach cruise velo
    * Jerk: takes 0.1s to reach desired accel
    */
    configs.MotionMagic.MotionMagicCruiseVelocity = 5; //RPS
    configs.MotionMagic.MotionMagicAcceleration =  10; //Units.radiansPerSecondToRotationsPerMinute(MAX_ANGLE-MIN_ANGLE) / 60;
    configs.MotionMagic.MotionMagicJerk = configs.MotionMagic.MotionMagicAcceleration * 10;

    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftPositioner.getConfigurator().apply(configs);

    var rightConfigs = configs;
    rightConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightPositioner.getConfigurator().apply(rightConfigs);

  }

  //angle should be between -5 and 79 degrees or sum like that ask carson
  public void goToAngle(double angle){
    try {
      desiredAngle = angle;
      if(angle > MAX_ANGLE || angle < MIN_ANGLE){
        throw new IllegalArgumentException("Arm does not reach that angle");
      }

      MotionMagicVoltage request = new MotionMagicVoltage(0);
      
      //leftPositioner.setControl(request.withPosition(Units.degreesToRotations(angle)));
      rightPositioner.setControl(request.withPosition(Units.degreesToRotations(angle)));
      SmartDashboard.putNumber("position output", request.Position);
    } catch (IllegalArgumentException e) {
        System.out.println("Arm does not reach that angle");
    }
  }

  //Calculate the angle the arm needs to go to
  public double calcAngle(){
    desiredAngle = currAngle + vision.getTx();
    return desiredAngle;
  }

  /*Ensures angle desired is within limits, if not it throws an exception but doesn terminate program */
  public void pointAtTarget() throws Exception{
    try {
      if(calcAngle() > MAX_ANGLE)
        throw new Exception("Arm cannot Reach that high of an angle");
      
      if(calcAngle() > MIN_ANGLE)
        throw new Exception("Arm cannot reach that low of an angle");

      goToAngle(calcAngle());
    } catch (Exception e) {
      System.out.println("Arm does not reach that angle");
      e.printStackTrace();
    }
  }


  @Override
  public void periodic() {
    currAngle = Units.rotationsToDegrees(leftPositioner.getPosition().getValue()); //gets position of mechanism in rotations and turns it into degrees
    SmartDashboard.putNumber("Current Arm Angle", currAngle);
    SmartDashboard.putNumber("Curren rot arm", leftPositioner.getPosition().getValue());
    SmartDashboard.putNumber("Desired Angle", desiredAngle);
  }
}
