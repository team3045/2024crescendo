package frc.lib.math;

public class Conversions {
    
    /**
     * @param wheelRPS Wheel Velocity: (in Rotations per Second)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Velocity: (in Meters per Second)
     */
    public static double RPSToMPS(double wheelRPS, double circumference){
        double wheelMPS = wheelRPS * circumference;
        return wheelMPS;
    }

    /**
     * @param wheelMPS Wheel Velocity: (in Meters per Second)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Velocity: (in Rotations per Second)
     */
    public static double MPSToRPS(double wheelMPS, double circumference){
        double wheelRPS = wheelMPS / circumference;
        return wheelRPS;
    }

    /**
     * @param wheelRotations Wheel Position: (in Rotations)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Distance: (in Meters)
     */
    public static double rotationsToMeters(double wheelRotations, double circumference){
        double wheelMeters = wheelRotations * circumference;
        return wheelMeters;
    }

    /**
     * @param wheelMeters Wheel Distance: (in Meters)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Position: (in Rotations)
     */
    public static double metersToRotations(double wheelMeters, double circumference){
        double wheelRotations = wheelMeters / circumference;
        return wheelRotations;
    }
    
    /**
     * @param wheelMPSS WheelAcceleration (in Meters / seconds^2)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Acceleration: (In Rotations / second^2)
     */
    public static double MPSSToRPSS(double wheelMPSS, double circumference){
        double wheelRPSS = wheelMPSS / circumference;
        return wheelRPSS;
    }

    /**
     * @param wheelRPSS WheelAcceleration (in Rotations / seconds^2)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Acceleration: (In Meters / second^2)
     */
    public static double RPSSToMPSS(double wheelRPSS, double circumference){
        double wheelMPSS = wheelRPSS * circumference;
        return wheelMPSS;
    }
}