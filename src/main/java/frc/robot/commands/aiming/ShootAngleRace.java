// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.aiming;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.PositionerSub;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootAngleRace extends ParallelRaceGroup {
  /** Creates a new ShootAngleRace. */
  public ShootAngleRace(double angle, PositionerSub positionerSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ShootAngle(angle, positionerSub), new WaitCommand(1));
  }

  private class ShootAngle extends Command{
    private double desAngle;
    private PositionerSub positioner;
    public ShootAngle(double angle, PositionerSub positioner){
      desAngle = angle;
      this.positioner = positioner;

      addRequirements(positioner);
    }

    @Override
    public void execute(){
      positioner.goToAngle(desAngle);
    }

    @Override
    public boolean isFinished(){
      if(Math.abs(positioner.getPositionDeg()-desAngle) < 0.3){
        return true;
      }
      return false;
    }
  }
}
