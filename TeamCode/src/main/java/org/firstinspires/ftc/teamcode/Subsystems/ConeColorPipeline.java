package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Util.AllianceColor;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/**
 * This pipeline detects where the custom marker is.
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

    public enum ConeColor {
        GREEN(new Scalar(1,1,1), new Scalar(1,1,1)), //TODO: Calibrate HSV values for colors
        CYAN(new Scalar(2,2,2), new Scalar(2,2,2)),
        BROWN(new Scalar(3,3,3), new Scalar(3,3,3)),
        OTHER(new Scalar(0,0,0), new Scalar(0,0,0)); //leave OTHER as is
        public final Scalar lowHSV;
        public final Scalar highHSV;
        ConeColor(Scalar lowHSV, Scalar highHSV) { this.highHSV = highHSV; this.lowHSV = lowHSV;}
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

        Rect rectCrop = new Rect(0, 720, 1920, 360); //TODO: Calibrate crop constants if necessary
        Mat crop = new Mat(mask, rectCrop);


        if (crop.empty()) {
            return input;
        }

        Mat threshBrown = new Mat();
        Mat threshGreen = new Mat();
        Mat threshCyan = new Mat();

        Core.inRange(crop, ConeColor.BROWN.lowHSV, ConeColor.BROWN.highHSV, threshBrown);
        Core.inRange(crop, ConeColor.GREEN.lowHSV, ConeColor.GREEN.highHSV, threshGreen);
        Core.inRange(crop, ConeColor.CYAN.lowHSV, ConeColor.CYAN.highHSV, threshCyan);


        double left_x = 0.375 * CAMERA_WIDTH;
        double right_x = 0.625 * CAMERA_WIDTH;

        boolean brown = false;
        boolean green = false;
        boolean cyan = false;

        if(Core.sumElems(threshBrown).val[0] / rectCrop.area() / 255 > 0) {
            coneColor = ConeColor.BROWN;
        }
        if(Core.sumElems(threshGreen).val[0] / rectCrop.area() / 255 > 0) {
            coneColor = ConeColor.GREEN;
        }
        if(Core.sumElems(threshCyan).val[0] / rectCrop.area() / 255 > 0) {
            coneColor = ConeColor.CYAN;
        }
        return crop;
    }

    /**
     * Gets the Marker Location, might be not found because of the Search Status.
     *
     * @return Where the marker is.
     * @see ConeColor
     */
    public ConeColorPipeline coneColorPipeline() {
        return this;
    }
}
