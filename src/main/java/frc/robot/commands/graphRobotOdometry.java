// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class graphRobotOdometry extends CommandBase {
  Swerve s_Swerve;

  private final Field2d robotField2d;

  double[] xPos = new double[10];
  double[] yPos = new double[10];

  /** Creates a new graphRobotOdometry. */
  public graphRobotOdometry(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    robotField2d = new Field2d();

    for(int i = 0; i < xPos.length; i++){
      xPos[i] = 0;
      yPos[i] = 0;
    }
  }

  //updates the array removing the least recent position (last index) and adding the most recent position at index 0
  //want to try and make something that draws the robots line as the match progresses
  //originally though could use graphs in smart dashboard but they dont work like that
  //looking into java.awt.graphics2d class
  
  public void updateArrayPositions(){
    //the coordinates that are about to be removed, used if want to delete lines
    double tempX = xPos[9], tempY = yPos[9];
    
    for(int i = xPos.length - 1; i >= 0; i++){
      //sets index 0 to the current position of robot (x,y)
      if(i-1 < 0){
        xPos[i] = s_Swerve.swerveOdometry.getPoseMeters().getX();
        yPos[i] = s_Swerve.swerveOdometry.getPoseMeters().getY();
      }
      //moves all elements one to the right, getting rid of the last element
      xPos[i] = xPos[i-1];
      yPos[i] = yPos[i-1];
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //puts field2d on smartdashboard
    SmartDashboard.putData("Robot Graph", robotField2d);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //updates robotPos on smartdashboard everytime run
    robotField2d.setRobotPose(s_Swerve.swerveOdometry.getPoseMeters());
    SmartDashboard.updateValues();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
