package frc.swervemath.Hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class WCPSwerve implements WheelHardware {
    private final TalonFX angle;
    private final TalonFX speed;

    int id;

    private final CANCoder encoder;
    private final float offset;

    private float setpoint;

    //maybe make not constant so that it works better with other ratios
    private final float angleToWheelRatio = 72/7;

    public WCPSwerve(int angleID, int speedID, int encoderID, int id){
        this.id = id;
        angle = new TalonFX(angleID);
        speed = new TalonFX(speedID);

        encoder = new CANCoder(encoderID);
        offset = (float)encoder.getAbsolutePosition();

        setpoint = 0;
    }
    public float getEncoderOut(){
        // %360+360%360 keeps it in the range from 0 to 360
        return ((((float)encoder.getAbsolutePosition()-offset)%360)+360)%360;
    }
    float angleFactor(){
        float delta = getEncoderOut()-setpoint;
        //convert to radians
        delta *= Math.PI/180;
        return (float)-Math.cos(delta);
    }
    public float PIDEncOut(){
        //%180 in because it works with the wheels backwards as well
        return getEncoderOut() % 180;
    }
    public float getError(){
        //%180 in because it works with the wheels backwards as well
        setpoint %= 180;
    
        //Checks what the error would be if looped over 180 in each direction
        float loopDownError = -((180 - setpoint) + PIDEncOut());
        float loopUpError = ((180 - PIDEncOut()) + setpoint);
    
        //finds the smaller of the two loop errors
        float loopError = Math.abs(loopUpError) > Math.abs(loopDownError) ? loopDownError : loopUpError;
    
        //finds the error if it doesn't loop
        float nonLoopError = (setpoint%180) - PIDEncOut();
        
        //finds the smaller of the loop and non-loop errors
        float error = (Math.abs(nonLoopError) > Math.abs(loopError)) ? loopError : nonLoopError;
    
        return (error);
      }
    public void setSpeed(float speed){
        //multiplies by the cosine of the error so that it goes in the opposite direction when the wheel is facing the other way
        this.speed.set(ControlMode.PercentOutput, speed * angleFactor());
    }
    
    //May be better to refactor with new TalonFX closed-loop PID controller
    public void setSetpoint(float setpoint){
        //converts from radians to degrees
        setpoint *= 180/(float)Math.PI;
        this.setpoint = setpoint;

        //sets the appropriate position for using the built in PID of the angle motor by multiplying the error by the gear ratio from the angle motor to the wheel
        angle.set(ControlMode.Position, angle.getSelectedSensorPosition() - (getError() * angleToWheelRatio * 360));
    }
}
