package frc.swervemath;

import frc.swervemath.Hardware.WCPSwerve;
import frc.swervemath.Hardware.WheelHardware;
import frc.swervemath.math.Vector2;

//Probably should be refactored as an interface to reduce overhead
public class Wheel { 
    public final WCPSwerve hardware;
    public final Vector2 pos;

    public void setAngleAndSpeed(float angle, float speed){
        hardware.setSetpoint(angle); // it is a WCPSwerve now so it has access to the setSetpoint and setSpeed methods
        hardware.setSpeed(speed);
    }
    public Wheel(WCPSwerve hardware, Vector2 pos){
        this.hardware = hardware;
        this.pos = pos;
    }
}
