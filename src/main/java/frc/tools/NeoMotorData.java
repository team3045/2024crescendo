package frc.tools;

import frc.tools.Book;

public class NeoMotorData extends MotorCurveData {
    public static NeoMotorData instance = new NeoMotorData();
    public NeoMotorData(){
        data = CSVReader.readBooksFromCSV("MotorCurve-12V.csv");
    }
    public static Book getAtSpeed(float speed) throws Exception {
        return instance.getAtSpeed_(speed);
    }
}
