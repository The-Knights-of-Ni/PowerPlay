package org.firstinspires.ftc.teamcode.Util.FileParser;

import android.util.Log;

import java.io.FileReader;
import java.io.IOException;

public abstract class File {
    public File() {
    }

    public abstract void load(String path) throws IOException;

    public String open(String path) throws IOException {
        java.io.File file = new java.io.File(path);
        FileReader fr = new FileReader(file);
        char[] a = new char[50];
        int result = fr.read(a); // reads the content to the array
        Log.d("Result: ", ((Integer) result).toString());
        StringBuilder output = new StringBuilder();

        for (char c : a) output.append(c); // prints the characters one by one
        fr.close();
        return output.toString();
    }
}
