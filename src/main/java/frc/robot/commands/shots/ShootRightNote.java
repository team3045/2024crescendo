// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shots;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.aiming.ShootAngleRace;
import frc.robot.subsystems.PositionerSub;
import frc.robot.subsystems.ShooterSub;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootRightNote extends SequentialCommandGroup {
  /** Creates a new ShootRightNote. */
  public ShootRightNote(ShooterSub shooter, PositionerSub arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelDeadlineGroup(
        new ShootAngleRace(PositionerSub.RIGHT_NOTE, arm),
        new InstantCommand(() -> shooter.shootPct())),
      new InstantCommand(() -> shooter.feed())
        .andThen(new WaitCommand(0.5))
        .andThen(new InstantCommand(() -> shooter.stopFeed()))
    );
  }
}
