package frc.tools;

import java.util.List;
import frc.tools.Book;

public class MotorCurveData {
    public List<Book> data;
    public Book getAtSpeed_(float speed) throws Exception {
        for (Book book : data) {
            if(book.speed > speed)
                return book;
        }
        throw new Exception("speed too high");
    }
    public enum MotorType {
        FALCON_500,
        NEO
    }
}
