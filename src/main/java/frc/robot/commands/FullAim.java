// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AutoSub;
import frc.robot.subsystems.LimeLightSub;
import frc.robot.subsystems.PositionerSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FullAim extends SequentialCommandGroup {
  /** Creates a new FullAim. */
  public FullAim(PositionerSub arm, LimeLightSub vision, ShooterSub shooter, Swerve swerve, AutoSub auto) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new TurnToTarget(vision, swerve, auto),new FindSpeakerAprilTag(arm, vision),new PointAtTarget(vision, arm));
  }
}
