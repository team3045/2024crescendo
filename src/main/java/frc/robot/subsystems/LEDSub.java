// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.intaking.IntakeNote;

public class LEDSub extends SubsystemBase {
  private static final CANdle candle = new CANdle(0, "Canivore 3045");

  //Team Colors
  public static final Color GREEN = new Color(0, 255, 0);
  public static final Color BLACK = new Color(0,0,0);

  //Robot State Colors TODO: set these
  public static final Color INTAKING = new Color(); 
  public static final Color CLIMBING = new Color();
  public static final Color HAS_PIECE = new Color();
  public static final Color READY_TO_SHOOT = new Color();

  //Match State Colors TODO: set these
  public static final Color END_GAME = new Color();
  
  /** Creates a new LEDSub. */
  public LEDSub() {
    configCandle();
    setDefaultCommand(runOnce(() -> setDefaultState()));
  }

  public void configCandle(){
    CANdleConfiguration config = new CANdleConfiguration();
    config.disableWhenLOS = true;
    config.statusLedOffWhenActive = true;
    config.v5Enabled = false;
    config.stripType = LEDStripType.RGB;
    config.brightnessScalar = 1.0;
    config.vBatOutputMode = VBatOutputMode.Modulated;
  }

  public void setDefaultState(){
    ColorFlowAnimation defaultAnimation = new ColorFlowAnimation((int)GREEN.red,(int)GREEN.green,(int)GREEN.blue);
    candle.clearAnimation(0);
    candle.animate(defaultAnimation);
  }

  public void setIntaking(){
    StrobeAnimation intakingAnimation = new StrobeAnimation(0, 0, 255);
    candle.clearAnimation(0);
    candle.animate(intakingAnimation);
  }

  public void setHasPiece(){
    ColorFlowAnimation hasPieceAnimation = new ColorFlowAnimation(255, 165, 0);
    candle.clearAnimation(0);
    candle.animate(hasPieceAnimation);
  }

  public void setTargetting(){
    LarsonAnimation targettingAnimation = new LarsonAnimation(255, 255, 0);
    candle.clearAnimation(0);
    candle.animate(targettingAnimation);
  }

  public void setTargetLock(){
    RainbowAnimation targetLockAnimation = new RainbowAnimation();
    candle.clearAnimation(0);
    candle.animate(targetLockAnimation);
  }
}
