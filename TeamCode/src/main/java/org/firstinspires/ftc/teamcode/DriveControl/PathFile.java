package org.firstinspires.ftc.teamcode.DriveControl;

import org.firstinspires.ftc.teamcode.Util.Coordinate;

import java.io.*;
import java.util.ArrayList;

public class PathFile {
    public static final int VERSION = 0;
    public static Path read(File file) throws IOException {
        FileInputStream fs = new FileInputStream(file);
        ArrayList<Integer> contents = new ArrayList<>();
        int r;
        while((r=fs.read())!=-1) {
            contents.add(r);
        }
        if (contents.get(0) > VERSION)
            throw new UnsupportedEncodingException("Unsupported Version of pathfile Current Version: " + VERSION + " File Version: " + contents.get(0));
        contents.remove(0);
        ArrayList<Waypoint> waypointArrayList = new ArrayList<>();
        int numberOfInfoBlocks = contents.get(0);
        contents.remove(0);
        int numOfX = contents.get(0);
        contents.remove(0);
        int numOfY = contents.get(0);
        contents.remove(0);
        if (numberOfInfoBlocks-2 > 1) {
            contents.subList(0, (numberOfInfoBlocks - 3)).clear();
        }
        int counter = 0;
        boolean stop;
        int x = 0;
        int y = 0;
        int chunkSize = numOfX + numOfY + 1;
        for (int i:contents) {
            int which = counter % chunkSize;
            if (which<numOfX) {
                x += i;
            }
            else if (which == (chunkSize-1)) {
                stop = i % 2 == 1;
                waypointArrayList.add(new Waypoint(new Coordinate(x, y), stop));
                x = 0;
                y = 0;
            }
            else {
                y += i;
            }
            counter++;
        }
        return new Path(waypointArrayList);
    }

    public static void write(File file, Path path) throws IOException {
        FileOutputStream fo = new FileOutputStream(file);
        ArrayList<Byte> arrayOut = new ArrayList<>();
        arrayOut.add((byte) VERSION);
        arrayOut.add((byte) 2);
        byte xMax = 32;
        byte yMax = 32;
        // TODO: Find the actual numbers
        arrayOut.add(xMax);
        arrayOut.add(yMax);
        for (Waypoint waypoint:path.waypoints) {
            int x = (int) waypoint.coordinate.getX();
            int y = (int) waypoint.coordinate.getY();
            int counter = 0;
            while (x>255) {
                arrayOut.add((byte) 255);
                x -= 255;
                counter++;
            }
            arrayOut.add((byte) x);
            counter++;

            if (xMax < counter) {
                throw new RuntimeException("x is too large. x="+ waypoint.coordinate.getX() + " xMax="+(xMax*255) + " counter=" + counter);
            }
            else {
                while (counter<xMax) {
                    arrayOut.add((byte) 0);
                    counter++;
                }
            }

            counter = 0;
            while (y > 255) {
                arrayOut.add((byte) 255);
                y -= 255;
                counter++;
            }
            arrayOut.add((byte) y);
            counter++;

            if (yMax < counter) {
                throw new RuntimeException("y is too large. y="+ waypoint.coordinate.getY() + " yMax="+(yMax*255) + " counter=" + counter);
            }
            else {
                while (counter<yMax) {
                    arrayOut.add((byte) 0);
                    counter++;
                }
            }


            if (waypoint.fullStop) {
                arrayOut.add((byte) 1);
            }
            else {
                arrayOut.add((byte) 0);
            }
        }
        byte[] output = new byte[arrayOut.size()];
        int count = 0;
        for (byte b: arrayOut) {
            output[count] = b;
            count++;
        }
        fo.write(output);
    }
}
