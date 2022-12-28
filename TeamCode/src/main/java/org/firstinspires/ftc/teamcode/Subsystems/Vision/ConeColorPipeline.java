package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Util.AllianceColor;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Core;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/**
 * This pipeline detects where the cone is.
 *
 * <p>It does this by splitting the camera input into 3 parts, the Left, Middle, and Right. It
 * checks each part for a custom marker (which is set to be green in the code), or some blue or red
 * tape, dependant on the alliance color. The marker is assumed to be yellow.</p>
 *
 * @see OpenCvPipeline
 * @see Vision
 */
public class ConeColorPipeline extends OpenCvPipeline {
    private final AllianceColor allianceColor;
    private final int CAMERA_WIDTH;
    private final int CAMERA_HEIGHT;
    private ConeColor coneColor = ConeColor.OTHER;


    /**
     * The cone color with the hsv constants
     */
    public enum ConeColor {
        GREEN(new Scalar(50,80,20), new Scalar(90,255,255), "Green"), // TODO:Calibrate color constants for Magenta

//        GREEN(new Scalar(50,0,0), new Scalar(70,180,230), "Green"), // TODO:Calibrate color constants for Magenta
        ORANGE(new Scalar(5, 200, 200), new Scalar(15, 235, 255), "Orange"),
        PINK(new Scalar(165,150,200), new Scalar(175,260,255), "Pink"),
        OTHER(new Scalar(0,0,0), new Scalar(0,0,0), "Other"); // leave OTHER as is
        public final Scalar lowHSV;
        public final Scalar highHSV;
        public final String color;
        ConeColor(Scalar lowHSV, Scalar highHSV, String color) { this.highHSV = highHSV; this.lowHSV = lowHSV; this.color = color;}

        @Override
        public String toString() {
            return "ConeColor{" +
                    "lowHSV=" + lowHSV +
                    ", highHSV=" + highHSV +
                    '}';
        }
    }

    /**
     * Class instantiation
     *
     * @see Robot
     * @see Telemetry
     * @see AllianceColor
     */
    public ConeColorPipeline(AllianceColor allianceColor, int width, int height) {
        this.allianceColor = allianceColor;
        this.CAMERA_WIDTH = width;
        this.CAMERA_HEIGHT = height;
    }

    /**
     * This method detects where the marker is.
     *
     * <p>It does this by splitting the camera input into left, right, and middle rectangles, these
     * rectangles need to be calibrated. Combined, they do not have to encompass the whole camera
     * input, they probably will only check a small part of it. We then assume the alliance color is
     * either (255, 0, 0) or (0, 0, 255), we get the info when the object is instantiated ({@link
     * #allianceColor}), and that the marker color is (0, 255, 0), which is a bright green ({@link
     * Scalar}'s are used for colors). We compare the marker color with the alliance color on each of
     * the rectangles, if the marker color is on none or multiple of them, it is marked as {@link
     * ConeColor#OTHER}, if otherwise, the respective Location it is in is returned via a
     * {@link ConeColor} variable called {@link #coneColor}
     *
     * @param input A Mask (the class is called {@link Mat})
     * @return The marker location
     * @see #allianceColor
     * @see Mat
     * @see Scalar
     * @see ConeColor
     */
    @Override
    public Mat processFrame(Mat input) {
        Mat mask = new Mat();
        Imgproc.cvtColor(input, mask, Imgproc.COLOR_RGB2HSV);

        // Find all pixels within given threshold color values
        Mat threshMagenta = new Mat();
        Mat threshGreen = new Mat();
        Mat threshOrange = new Mat();

        Core.inRange(mask, ConeColor.PINK.lowHSV, ConeColor.PINK.highHSV, threshMagenta);
        Core.inRange(mask, ConeColor.GREEN.lowHSV, ConeColor.GREEN.highHSV, threshGreen);
        Core.inRange(mask, ConeColor.ORANGE.lowHSV, ConeColor.ORANGE.highHSV, threshOrange);

        // Select the mat which has the most white pixels
        double magentaSum = Core.sumElems(threshMagenta).val[0] / (CAMERA_HEIGHT*CAMERA_WIDTH) / 255;
        double greenSum = Core.sumElems(threshGreen).val[0] / (CAMERA_HEIGHT*CAMERA_WIDTH) / 255;
        double cyanSum = Core.sumElems(threshOrange).val[0] / (CAMERA_HEIGHT * CAMERA_WIDTH) / 255;

        if(magentaSum > 0 || greenSum > 0 || cyanSum > 0) {
            if(magentaSum >= greenSum && magentaSum >= cyanSum) {
                coneColor = ConeColor.PINK;
            } else if(greenSum >= cyanSum) {
                coneColor = ConeColor.GREEN;
            } else {
                coneColor = ConeColor.ORANGE;
            }
        } else {
            coneColor = ConeColor.OTHER;
        }

        // Return whichever mat is desired to be viewed on Camera Stream
        return threshGreen;
    }

    /**
     * Gets the Marker Location, might be not found because of the Search Status.
     *
     * @return Where the marker is.
     * @see ConeColor
     */
    public ConeColor getConeColor() {
        return coneColor;
    }
}