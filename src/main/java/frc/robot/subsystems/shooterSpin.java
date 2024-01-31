package frc.robot.subsystems;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.FindCirclePoint;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import java.util.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class shooterSpin extends SubsystemBase{
    public static CANSparkMax m1;
    public static CANSparkMax m2;


    public shooterSpin (int m1, int m2) {
        this.m1 = new CANSparkMax(m1, MotorType.kBrushless);
        this.m2 = new CANSparkMax(m2, MotorType.kBrushless);
    }

    public Command spin() {
        m1.set(Constants.SHOOTERSPEEDTOP);
        m2.set(Constants.SHOOTERSPEEDBOTTOM);
        return null;
    }
    public Command stop () {
        m1.stop(); m2.stop();
        return null;
    }

    public static void setTopSpeed(double speed) {
        m1.set(speed);
    }

    public static void setBottomSpeed(double speed) {
        m2.set(speed);
    }

    @Override
    public void periodic() {
        while(RobotContainer.getInstance().buttonBoard.getRawButton(Constants.SHOOTERSPINBUTTON)) {m1.set(Constants.SHOOTERSPEEDTOP);m2.set(Constants.SHOOTERSPEEDBOTTOM);}
        while(!RobotContainer.getInstance().buttonBoard.getRawButton(Constants.SHOOTERSPINBUTTON)) {this.stop();}
    }
}
