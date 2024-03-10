// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intaking;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PositionerSub;
import frc.robot.subsystems.ShooterSub;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FullIntake extends SequentialCommandGroup {
  public static IntakeNote intakeNote;
  public static int count = 1;
  /** Creates a new FullIntake. */
  public FullIntake(PositionerSub arm, Intake intake, ShooterSub shooter) {
    count++;
    if(count % 2 == 0){
      System.out.println("full intake");
      //addCommands(new InstantCommand(() -> arm.goToIntake()), new IntakeNote(intake, shooter));
    }
    else
      addCommands(new StopIntake(intake,shooter));

  }
}
