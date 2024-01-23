package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import java.util.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.enums.*;

public class ClimbingArms extends SubsystemBase{
    private final CANSparkMax armMotor1;
    private final CANSparkMax armMotor2;

    public ClimbingArms(int armMotor1, int armMotor2) {
        this.armMotor1 = new CANSparkMax(armMotor1, MotorType.kBrushless);
        this.armMotor2 = new CANSparkMax(armMotor2, MotorType.kBrushless);
// use pheonix 6 api no pid calculate how 4096 or 2048 encoder rotations = 1 rotation -> calculate circumference and then use that to get the change in hight
        // highest it can go * however many encoder measurements = the # of encoder rotations we need to get to hight $

    }

    @Override
    public void periodic() {
        while (true) {
            while (RobotContainer.getInstance().buttonBoard.getRawButton(Constants.climbingArmUpButton)) {armMotor1.set(Constants.climbingArmSpeed);armMotor2.set(-Constants.climbingArmSpeed);}
            while (RobotContainer.getInstance().buttonBoard.getRawButton(Constants.climbingArmDownButton)) {armMotor1.set(-Constants.climbingArmSpeed);armMotor2.set(Constants.climbingArmSpeed);}
            while (!RobotContainer.getInstance().buttonBoard.getRawButton(Constants.climbingArmUpButton) && !RobotContainer.getInstance().buttonBoard.getRawButton(Constants.climbingArmDownButton)) {armMotor1.stopMotor(); armMotor2.stopMotor();}
        }
        /*
        armMotor2.set(-(RobotContainer.getInstance().buttonBoard.getRawButton(Constants.climbingArmUpButton) ? Constants.climbingArmSpeed : 0) -
                    (RobotContainer.getInstance().buttonBoard.getRawButton(Constants.climbingArmDownButton) ? -Constants.climbingArmSpeed : 0));

        armMotor1.set((RobotContainer.getInstance().buttonBoard.getRawButton(Constants.climbingArmUpButton) ? Constants.climbingArmSpeed : 0) +
                      (RobotContainer.getInstance().buttonBoard.getRawButton(Constants.climbingArmDownButton) ? -Constants.climbingArmSpeed : 0));
   */ }




}