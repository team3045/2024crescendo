package frc.tools;

import frc.tools.Book;

public class Falcon500MotorData extends MotorCurveData {
    public static Falcon500MotorData instance = new Falcon500MotorData();
    public Falcon500MotorData(){
        data = CSVReader.readBooksFromCSV("Falcon500MotorCurveData.csv");
    }
    public static Book getAtSpeed(float speed) throws Exception{
        return instance.getAtSpeed_(speed);
    }
}
