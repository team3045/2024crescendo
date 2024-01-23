package frc.robot.subsystems;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.FindCirclePoint;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import java.util.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
public class shooterRotation extends SubsystemBase {
    private final CANSparkMax motor;

    public shooterRotation (int motor) {
        this.motor = new CANSparkMax(motor, MotorType.kBrushless);
    }

    // for every x radians rotated of the motor, x* ROTATIONGEARCHAINCONNECTIONCIRCUMFERENCE/ROTATIONGEARCIRCUMFERENCE rad of the big gear asnd, by extention, the shooter
    public double bigToSmall (double big) {
        return big * Constants.ROTATIONGEARCIRCUMFERENCE / Constants.ROTATIONGEARCHAINCONNECTIONCIRCUMFERENCE;
    }
    public double smallToBig (double small) {
        return small * Constants.ROTATIONGEARCHAINCONNECTIONCIRCUMFERENCE / Constants.ROTATIONGEARCIRCUMFERENCE;
    }

    @Override
    public void periodic () {
        while (RobotContainer.getInstance().buttonBoard.getRawButton(Constants.SHOOTERDOWNBUTTON)) {this.motor.set(-Constants.SHOOTERMOTORSPEED);}
        while (RobotContainer.getInstance().buttonBoard.getRawButton(Constants.SHOOTERUPBUTTON)) {this.motor.set(Constants.SHOOTERMOTORSPEED);}
        while (!RobotContainer.getInstance().buttonBoard.getRawButton(Constants.SHOOTERDOWNBUTTON && !RobotContainer.getInstance().buttonBoard.getRawButton(Constants.SHOOTERUPBUTTON) {this.motor.stop();}
    }

    // set to angle? add if needed.
}
