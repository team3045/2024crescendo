// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimeLightSub;
import frc.robot.subsystems.PositionerSub;

public class PointAtTarget extends Command {
  private LimeLightSub vision;
  private PositionerSub arm;

  /*How much higher we have to aim to get the shooter
   to see the point rather than just camera 
   Determined by testing rather than mathematically*/
  private static final double camDegOffset = 0; //0 once we mount in new spot

  private double desiredAng;
  /** Creates a new PointAtTarget. */
  public PointAtTarget(LimeLightSub vision, PositionerSub arm) {
    this.vision = vision;
    this.arm = arm;
    
    desiredAng = 0;
    addRequirements(arm);
  }

  /*Regression to calculate angle based on distance */
  // public double calcAngle(){
  //   double distance = vision.getHorizDistanceSpeaker();

  //   /*break out of function if target not seen */
  //   if(distance == -1){
  //     System.out.println("target not seen, cant move arm");
  //     return -1;
  //   }

  //   /*Regression for angle, currently arbitray values */
  //   double angle = 3 * Math.pow(distance, 3);
  //   angle += 2 * Math.pow(distance, 2);
  //   angle += 1.5 * distance;
  //   angle += 5;

  //   return angle;
  // }

  public void calcAngle(){
    vision.setAimingPipeline();
    double currAngle = arm.getPositionDeg();
    double diff = vision.getTy();

    double desiredDeg =currAngle+diff+camDegOffset;

    if(diff != 0){
      desiredAng = desiredDeg;
    }
    else
      desiredAng = arm.getPositionDeg();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    calcAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
      //calc angle everytime so hopefulyl you can hold down and it adjusts while moving
      //PRECONDITION: CAN SEE APRILTAG
      //POSTCONDITION: POINTS SHOOTER AT DESIRED OFFSET FROM APRILTAG
      calcAngle();
      arm.goToAngle(desiredAng);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    vision.setLocalizerPipeline();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
