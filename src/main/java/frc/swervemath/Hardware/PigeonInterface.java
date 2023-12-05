package frc.swervemath.Hardware;

import com.ctre.phoenix.sensors.Pigeon2;

import frc.robot.Constants;

//should refactor name because it isn't an interface
public class PigeonInterface implements Gyro {
    private final Pigeon2 pigeon;
    private float calibrationAngle;

    public PigeonInterface(int pigID){
        pigeon = new Pigeon2(pigID);
    }
    public void zero(){
        calibrationAngle = (float)pigeon.getYaw();
    }
    public float angleInRads(){
        //takes angle and converts it to radians
        return ((float)(-(pigeon.getYaw())) - calibrationAngle) * (float)Math.PI/180;
    }
}
