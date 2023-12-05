package frc.tools;

public class Book {
    public float speed;
    public float torque;
    public float current;
    public float suppliedPower;
    public float outputPower;
    public float efficiency;
    public float powerDissipation;

    public Book(float speed, float torque, float current, float suppliedPower, float outputPower, float efficiency, float powerDissipation){
        this.speed = speed;
        this.torque = torque;
        this.current = current;
        this.suppliedPower = suppliedPower;
        this.outputPower = outputPower;
        this.efficiency = efficiency;
        this.powerDissipation = powerDissipation;
    }
}