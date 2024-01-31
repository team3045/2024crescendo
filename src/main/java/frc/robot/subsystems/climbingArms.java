package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.enums.*;

public class ClimbingArms extends SubsystemBase{
    public final CANSparkMax armMotor1;
    public final CANSparkMax armMotor2;

    public ClimbingArms(int armMotor1, int armMotor2) {
        this.armMotor1 = new CANSparkMax(armMotor1, MotorType.kBrushless);
        this.armMotor2 = new CANSparkMax(armMotor2, MotorType.kBrushless);
// use pheonix 6 api no pid calculate how 4096 or 2048 encoder rotations = 1 rotation -> calculate circumference and then use that to get the change in hight
        // highest it can go * however many encoder measurements = the # of encoder rotations we need to get to hight $

    }

    public Command leftArm (int i) {
        this.armMotor1.set(Constants.CLIMBINGARMSPEED * i);
        return null;
    }
    public Command rightArm (int i) { // make i 1 or -1 based on if it is supposed to go up or down
        this.armMotor2.set(Constants.CLIMBINGARMSPEED * i);
        return null;
    }

    public Command stopArms () {
        this.armMotor2.stopMotor();
        this.armMotor1.stopMotor();
        return null;
    }

    @Override
    public void periodic() {
        while (true) {
            while (RobotContainer.getInstance().buttonBoard.getRawButton(Constants.ARMSUPBUTTON)) {
                if (RobotContainer) armMotor1.set(Constants.CLIMBINGARMSPEED);
                armMotor2.set(-Constants.CLIMBINGARMSPEED);
            }
            while (RobotContainer.getInstance().buttonBoard.getRawButton(Constants.ARMSDOWNBUTTON)) {armMotor1.set(-Constants.CLIMBINGARMSPEED);armMotor2.set(Constants.CLIMBINGARMSPEED);}
            while (!RobotContainer.getInstance().buttonBoard.getRawButton(Constants.ARMSUPBUTTON) && !RobotContainer.getInstance().buttonBoard.getRawButton(Constants.ARMSDOWNBUTTON)) {armMotor1.stopMotor(); armMotor2.stopMotor();}
        }
        /*
        armMotor2.set(-(RobotContainer.getInstance().buttonBoard.getRawButton(Constants.climbingArmUpButton) ? Constants.climbingArmSpeed : 0) -
                    (RobotContainer.getInstance().buttonBoard.getRawButton(Constants.climbingArmDownButton) ? -Constants.climbingArmSpeed : 0));

        armMotor1.set((RobotContainer.getInstance().buttonBoard.getRawButton(Constants.climbingArmUpButton) ? Constants.climbingArmSpeed : 0) +
                      (RobotContainer.getInstance().buttonBoard.getRawButton(Constants.climbingArmDownButton) ? -Constants.climbingArmSpeed : 0));
   */ }




}