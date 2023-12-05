package frc.swervemath;

import javax.swing.GrayFilter;

import frc.swervemath.Hardware.Gyro;
import frc.swervemath.math.Vector2;
import frc.swervemath.math.Vector3;

public class DriveTrain {
    public final Wheel[] wheels;
    public final Gyro gyro;

    public DriveTrain(Gyro gyro, Wheel... wheels){
        this.wheels = wheels;
        this.gyro = gyro;
    }

    public void robotOrientedDrive(Vector2 Heading, float rot){
        if(rot == 0){
            //Without rotation, there isn't a point to be tangent to, just a direction to move
            float speed = Heading.magnitude();

            //maxes the speed out at 100%
            speed = speed > 1 ? 1 : speed;
            for(int i = 0; i < wheels.length; i++){
                wheels[i].setAngleAndSpeed(-(float)Math.atan2(Heading.y, Heading.x) + (float)Math.PI / 2, speed);
            }
        } else {
            //balances rotation with strafing and makes the point on a line perpendicular to the heading to control the direction of movement
            Vector2 pointOfRot = Vector2.multiply(new Vector2(Heading.y, -Heading.x), 1 / rot);

            float max = 0;

            float[] angles = new float[wheels.length];
            float[] mags = new float[wheels.length];

            for(int i = 0; i < wheels.length; i ++){
                Wheel wheel = wheels[i];

                //Rotates the delta 90 degrees dependent on the intended direction to spin
                if(rot > 0)
                    angles[i] = (float)Math.atan2(-pointOfRot.x + wheel.pos.x, -(-pointOfRot.y + wheel.pos.y));
                else
                    angles[i] = (float)Math.atan2(-(-pointOfRot.x + wheel.pos.x), -pointOfRot.y + wheel.pos.y);
                mags[i] = Vector2.subtract(wheel.pos, pointOfRot).magnitude();

                //Moves from standard position to the angle that we use
                angles[i] *= -1;
                angles[i] += Math.PI / 2;

                //finds the maximum distance from the point of rot to a wheel
                max = (mags[i] > max) ? mags[i] : max;
            }
            float speed = new Vector3(Heading.x, Heading.y, rot).magnitude();

            //maxes the speed out at 100%
            speed = speed > 1 ? 1 : speed;

            for(int i = 0; i < wheels.length; i ++) {
                wheels[i].setAngleAndSpeed(angles[i], (mags[i] / max) * speed); 
            }
        }
    }
    public void fieldOrientedDrive(Vector2 Heading, float rot){
        //rotates the heading input by the negative rotation of the bot to ensure that it stays field-oriented and then puts that into the robot oriented drive method
        float cos = (float)Math.cos(-gyro.angleInRads());
        float sin = (float)Math.sin(-gyro.angleInRads());

        System.out.println("Angle of Bot: " + gyro.angleInRads() * 180/Math.PI);

        robotOrientedDrive(new Vector2(Heading.x * cos - Heading.y * sin, Heading.y * cos + Heading.x * sin), rot);
    }
    public Gyro getGyro(){
        return gyro;
    }
}
