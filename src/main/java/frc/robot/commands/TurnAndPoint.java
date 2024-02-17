// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.AutoSub;
import frc.robot.subsystems.LimeLightSub;
import frc.robot.subsystems.PositionerSub;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnAndPoint extends ParallelCommandGroup {
  /** Creates a new TurnAndPoint. */
  public TurnAndPoint(Swerve swerve, LimeLightSub vision, PositionerSub arm, AutoSub auto) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new PointAtTarget(swerve, vision, arm), new TurnToTarget(vision, swerve, auto));
  }
}
