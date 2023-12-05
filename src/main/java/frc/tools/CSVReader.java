package frc.tools;

import java.io.BufferedReader;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Filesystem;

import frc.tools.Book;

public class CSVReader {
    public static List<Book> readBooksFromCSV(String fileName) {
        List<Book> books = new ArrayList<>();
        Path pathToFile = Filesystem.getDeployDirectory().toPath().resolve(fileName);


        // create an instance of BufferedReader
        // using try with resource, Java 7 feature to close resources
        try (BufferedReader br = Files.newBufferedReader(pathToFile,
                StandardCharsets.US_ASCII)) {

            //skips first line which won't parse
            br.readLine();
            
            // read the first line from the text file
            String line = br.readLine();

            // loop until all lines are read
            while (line != null) {

                // use string.split to load a string array with the values from
                // each line of
                // the file, using a comma as the delimiter
                String[] attributes = line.split(",");

                Book book = createBook(attributes);

                // adding book into ArrayList
                books.add(book);

                // read next line before looping
                // if end of file reached, line would be null
                line = br.readLine();
            }

        } catch (IOException ioe) {
            ioe.printStackTrace();
        }

        return books;
    }

    private static Book createBook(String[] metadata) {
        float speed = Float.parseFloat(metadata[0]);
        float torque = Float.parseFloat(metadata[1]);
        float current = Float.parseFloat(metadata[2]);
        float suppliedPower = Float.parseFloat(metadata[3]);
        float outputPower = Float.parseFloat(metadata[4]);
        float efficiency = Float.parseFloat(metadata[5]);
        float powerDissipation = Float.parseFloat(metadata[6]);

        return new Book(speed, torque, current, suppliedPower, outputPower, efficiency, powerDissipation);
    }
}
