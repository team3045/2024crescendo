// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PositionerSub extends SubsystemBase {
  private final CANcoder absEncoder = new CANcoder(20);
  private final double absEncoderOffset = Units.degreesToRotations(-166.552734375+15); //range 0-1, in rotations I presume
 
  private static final TalonFX leftPositioner = new TalonFX(17); //Turns counterclockwise to move armp up
  private static final TalonFX rightPositioner = new TalonFX(16); // turns clockwise to move arm up

  private LimeLightSub vision;

  private static final boolean absoluteEncoder = false;

  public static double currAngle;
  private static double desiredAngle;

  

  public static final double MIN_ANGLE = 16; //0.108264602706615
  public static final double MAX_ANGLE = 79; 
  public static final double INTAKE_ANGLE = Units.rotationsToDegrees(0.110494702762366);
  public static final double MIDDLE_NOTE = 33;
  public static final double RIGHT_NOTE = 31;
  public static final double SPEAKER_ANGLE = 56;
  public static final double AMP_ANGLE = 66;
  /** Creates a new PositionerSub. */
  public PositionerSub(LimeLightSub vision) {
    /*Initialize our limelight for the shooter 
     * Seperate from our limelight for localization
    */
    this.vision = vision;

    /*Config both positioners */
    configCanCoder();
    configMotors();

    /*Resets both values on boot, change if using absolute encoder 
     * if not using absolute it is necessary to zero the arm on boot
    */
    if(!absoluteEncoder){
      leftPositioner.setPosition(0);
      rightPositioner.setPosition(0);
    }
    else{
      leftPositioner.setPosition(getPositionRot());
      rightPositioner.setPosition(getPositionRot());
    }
    
    currAngle = Units.rotationsToDegrees(leftPositioner.getPosition().getValue()); //gets position of mechanism in rotations and turns it into degrees
    desiredAngle = currAngle; //sets desired angle to current angle so we dont move

    SmartDashboard.putNumber("Desired angle", 0);
    SmartDashboard.putNumber("kP", 60);
    SmartDashboard.putNumber("kI", 0);
    SmartDashboard.putNumber("kD", 0.1);
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

    configs.Feedback.FeedbackRemoteSensorID = 20;
    configs.Feedback.RotorToSensorRatio = (70 / 26) * (5) * (4) *(1/1.4); //* 1/1.4 to fix the gear ratio empiraclly */
    configs.Feedback.SensorToMechanismRatio = 1;
    configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    /* Rps at cruise velocity, should be in rotations of mechanism rather than just motor axle
    * Accel: Takes 0.5s to reach cruise velo
    * Jerk: takes 0.1s to reach desired accel
    */
    configs.MotionMagic.MotionMagicCruiseVelocity = 2;
    configs.MotionMagic.MotionMagicAcceleration =  4; //Units.radiansPerSecondToRotationsPerMinute(MAX_ANGLE-MIN_ANGLE) / 60;
    configs.MotionMagic.MotionMagicJerk = configs.MotionMagic.MotionMagicAcceleration * 10;

    configs.CurrentLimits.SupplyCurrentLimit = 40;
    configs.CurrentLimits.SupplyCurrentThreshold = 60;
    configs.CurrentLimits.SupplyTimeThreshold = 0.2;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;

    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftPositioner.getConfigurator().apply(configs);

    var rightConfigs = configs;
    rightConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightPositioner.getConfigurator().apply(rightConfigs);

  }

  public void configCanCoder(){
    var config = new CANcoderConfiguration();
    
    MagnetSensorConfigs sensorConfigs = config.MagnetSensor;

    sensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    sensorConfigs.MagnetOffset = absEncoderOffset;
    sensorConfigs.SensorDirection = SensorDirectionValue.Clockwise_Positive;

    absEncoder.getConfigurator().apply(config);
  }

  
  //angle should be between -5 and 79 degrees or sum like that ask carson
  public void goToAngle(double angle){
    desiredAngle = angle;

    if(angle > MAX_ANGLE)
      angle = MAX_ANGLE;
    if(angle < MIN_ANGLE)
      angle = MIN_ANGLE;
        
    MotionMagicVoltage request = new MotionMagicVoltage(0);
    

    leftPositioner.setControl(request.withPosition(Units.degreesToRotations(angle)));
    //rightPositioner.setControl(request.withPosition(Units.degreesToRotations(angle)));

    SmartDashboard.putNumber("position output", request.Position);
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

  public void goToIntake(){
    goToAngle(INTAKE_ANGLE);
  }
  
  public void goToSpeaker(){
    goToAngle(SPEAKER_ANGLE);
  }

  public void decreaseAngle(){
    goToAngle(getPositionDeg() - 2);
  }

  public double getPositionRot(){
    double position = absEncoder.getPosition().getValueAsDouble();

    return position;
  }

  public double getPositionDeg(){
    return Units.rotationsToDegrees(getPositionRot());
  }

  public void goToMiddleNote(){
    goToAngle(MIDDLE_NOTE);
  }

  public void goToRightNote(){
    goToAngle(RIGHT_NOTE);
  }


  @Override
  public void periodic() {
    currAngle = Units.rotationsToDegrees(leftPositioner.getPosition().getValue()); //gets position of mechanism in rotations and turns it into degrees
    SmartDashboard.putNumber("Current Arm Angle", getPositionDeg());
    SmartDashboard.putNumber("Curren rot arm", leftPositioner.getPosition().getValue());
    if(currAngle > MAX_ANGLE)
      goToAngle(MAX_ANGLE);
    if(currAngle < MIN_ANGLE)
      goToAngle(MIN_ANGLE);
    
    SmartDashboard.putNumber("Abs encoder", Units.rotationsToDegrees(absEncoder.getAbsolutePosition().getValueAsDouble()));
   
  }
 
}
