package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.swervemath.math.Vector2;
import frc.tools.*;

public class ArticulatedArm extends SubsystemBase {
    public final TalonFX firstStageMotor = new TalonFX(Constants.ArmConstants.firstStageMotorCANid);

    public final CANSparkMax secondStageMotor1 = new CANSparkMax(Constants.ArmConstants.secondStageMotorCANid1, MotorType.kBrushless);
    public final CANSparkMax secondStageMotor2 = new CANSparkMax(Constants.ArmConstants.secondStageMotorCANid2, MotorType.kBrushless);
    public final CANSparkMax intakeWristMotor = new CANSparkMax(Constants.ArmConstants.intakeWristMotorCANid, MotorType.kBrushless);

    public double firstStageTargetRot;
    public double secondStageTargetRot;
    public double intakeTargetRot;

    public double firstStageRot;
    public double secondStageRot;
    public double intakeRot;

    public PIDController firstStageController;
    public PIDController secondStageController;
    public PIDController intakeController;

    public ArticulatedArm(){
        firstStageController = new PIDController(Constants.ArmConstants.firstStageKp, Constants.ArmConstants.firstStageKi, Constants.ArmConstants.firstStageKd);
        firstStageController.setIntegratorRange(-Constants.ArmConstants.iLimit, Constants.ArmConstants.iLimit);

        secondStageController = new PIDController(Constants.ArmConstants.secondStageKp, Constants.ArmConstants.secondStageKi, Constants.ArmConstants.secondStageKd);
        secondStageController.setIntegratorRange(-Constants.ArmConstants.iLimit, Constants.ArmConstants.iLimit);

        intakeController = new PIDController(Constants.ArmConstants.intakeKp, Constants.ArmConstants.intakeKi, Constants.ArmConstants.intakeKd);
        intakeController.setIntegratorRange(-Constants.ArmConstants.iLimit, Constants.ArmConstants.iLimit);
    }

    public double getTorqueFirstStage(double firstStageRot, double secondStageRot, double intakeRot){
        secondStageRot += 180;

        Vector2 totalCenterOfMass = new Vector2(0, 0);
        float totalMass = 0;

        totalCenterOfMass = Vector2.add(totalCenterOfMass, new Vector2((float)Math.sin(firstStageRot * 180 / Math.PI) * Constants.ArmConstants.firstStageCenterOfMass, 
                                                                       (float)Math.cos(firstStageRot * 180 / Math.PI) * Constants.ArmConstants.firstStageCenterOfMass));
        totalMass += Constants.ArmConstants.firstStageMass;

        Vector2 pivotPoint = new Vector2((float)Math.sin(firstStageRot * 180 / Math.PI) * Constants.ArmConstants.firstStageNextPivot, 
                                         (float)Math.cos(firstStageRot * 180 / Math.PI) * Constants.ArmConstants.firstStageNextPivot);
        Vector2 transformedCenterOfMass = Vector2.add(pivotPoint, new Vector2((float)Math.sin(secondStageRot * 180 / Math.PI) * Constants.ArmConstants.secondStageCenterOfMass, 
                                                                              (float)Math.cos(secondStageRot * 180 / Math.PI) * Constants.ArmConstants.secondStageCenterOfMass));
        totalCenterOfMass = Vector2.add(totalCenterOfMass, transformedCenterOfMass);
        totalMass += Constants.ArmConstants.secondStageMass;

        pivotPoint = Vector2.add(pivotPoint, new Vector2((float)Math.sin(secondStageRot * 180 / Math.PI) * Constants.ArmConstants.secondStageNextPivot, 
                                                         (float)Math.cos(secondStageRot * 180 / Math.PI) * Constants.ArmConstants.secondStageNextPivot));

        transformedCenterOfMass = Vector2.add(pivotPoint, new Vector2((float)Math.sin(intakeRot * 180 / Math.PI) * Constants.ArmConstants.intakeCenterOfMass, 
                                                                      (float)Math.cos(intakeRot * 180 / Math.PI) * Constants.ArmConstants.intakeCenterOfMass));
        totalCenterOfMass = Vector2.add(totalCenterOfMass, transformedCenterOfMass);
        totalMass += Constants.ArmConstants.intakeMass;

        Vector2 centerOfMass = Vector2.divide(totalCenterOfMass, totalMass);
        return totalMass * Math.sin(Math.atan2(centerOfMass.y, centerOfMass.x)) * centerOfMass.magnitude() * 1.3558179483314004/12;
    }

    public double getTorqueSecondStage(double secondStageRot, double intakeRot){
        secondStageRot += 180;

        Vector2 totalCenterOfMass = new Vector2(0, 0);
        float totalMass = 0;

        Vector2 transformedCenterOfMass = new Vector2((float)Math.sin(secondStageRot * 180 / Math.PI) * Constants.ArmConstants.secondStageCenterOfMass, 
                                                      (float)Math.cos(secondStageRot * 180 / Math.PI) * Constants.ArmConstants.secondStageCenterOfMass);
        totalCenterOfMass = Vector2.add(totalCenterOfMass, transformedCenterOfMass);
        totalMass += Constants.ArmConstants.secondStageMass;

        Vector2 pivotPoint = new Vector2((float)Math.sin(secondStageRot * 180 / Math.PI) * Constants.ArmConstants.secondStageNextPivot, 
                                         (float)Math.cos(secondStageRot * 180 / Math.PI) * Constants.ArmConstants.secondStageNextPivot);

        transformedCenterOfMass = Vector2.add(pivotPoint, new Vector2((float)Math.sin(intakeRot * 180 / Math.PI) * Constants.ArmConstants.intakeCenterOfMass, 
                                                                      (float)Math.cos(intakeRot * 180 / Math.PI) * Constants.ArmConstants.intakeCenterOfMass));
        totalCenterOfMass = Vector2.add(totalCenterOfMass, transformedCenterOfMass);
        totalMass += Constants.ArmConstants.intakeMass;

        Vector2 centerOfMass = Vector2.divide(totalCenterOfMass, totalMass);
        return totalMass * Math.sin(Math.atan2(centerOfMass.y, centerOfMass.x)) * centerOfMass.magnitude() * 1.3558179483314004/12;
    }

    public double getTorqueIntake(double intakeRot){
        Vector2 totalCenterOfMass = new Vector2(0, 0);
        float totalMass = 0;

        Vector2 transformedCenterOfMass = new Vector2((float)Math.sin(intakeRot * 180 / Math.PI) * Constants.ArmConstants.intakeCenterOfMass, 
                                                      (float)Math.cos(intakeRot * 180 / Math.PI) * Constants.ArmConstants.intakeCenterOfMass);
        totalCenterOfMass = Vector2.add(totalCenterOfMass, transformedCenterOfMass);
        totalMass += Constants.ArmConstants.intakeMass;

        Vector2 centerOfMass = Vector2.divide(totalCenterOfMass, totalMass);
        return totalMass * Math.sin(Math.atan2(centerOfMass.y, centerOfMass.x)) * centerOfMass.magnitude() * 1.3558179483314004/12;
    }

    @Override
    public void periodic(){
        updateRots();
        try{
            firstStageMotor.set(TalonFXControlMode.Current, firstStageController.calculate(firstStageRot) + getAmpsForTorque(MotorCurveData.MotorType.FALCON_500, 
                                                                                                                             firstStageMotor.getSelectedSensorVelocity() * 600 / 2048, 
                                                                                                                             getTorqueFirstStage(firstStageRot, secondStageRot, intakeRot) / Constants.ArmConstants.firstStageGearRatio));

            secondStageMotor1.set((secondStageController.calculate(secondStageRot) + getAmpsForTorque(MotorCurveData.MotorType.NEO, 
                                                                                                      secondStageMotor1.getEncoder().getVelocity() * 600 / 42, 
                                                                                                      getTorqueSecondStage(secondStageRot, intakeRot) / 2 / Constants.ArmConstants.secondStageGearRatio)) 
                                  / Constants.ArmConstants.kSecondStageCurrentLimit);
            secondStageMotor2.set((secondStageController.calculate(secondStageRot) + getAmpsForTorque(MotorCurveData.MotorType.NEO, 
                                                                                                      secondStageMotor1.getEncoder().getVelocity() * 600 / 42, 
                                                                                                      getTorqueSecondStage(secondStageRot, intakeRot) / 2 / Constants.ArmConstants.secondStageGearRatio)) 
                                   / Constants.ArmConstants.kSecondStageCurrentLimit);

            intakeWristMotor.set((intakeController.calculate(intakeRot) + getAmpsForTorque(MotorCurveData.MotorType.NEO, 
                                                                                           intakeWristMotor.getEncoder().getVelocity() * 600 / 42, 
                                                                                           getTorqueIntake(intakeRot) / Constants.ArmConstants.intakeGearRatio)) 
                                 / Constants.ArmConstants.kSecondStageCurrentLimit);
        } catch(Exception e) {
            System.err.println(e.getMessage());
            
        }
    }
    double getAmpsForTorque(MotorCurveData.MotorType motorType, double rpm, double torque) throws Exception{
        Book book = null;
        switch(motorType){
            case FALCON_500:
                book = Falcon500MotorData.getAtSpeed((float)rpm);
                break;
            case NEO:
                book = NeoMotorData.getAtSpeed((float)rpm);
                break;
            default:
                break;
        }
        float amps = book.current;
        float nm = book.torque;

        return (double)((torque / nm) * amps);
    }
    void updateRots(){

    }
}
