package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Util.AllianceColor;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Core;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

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

    public enum ConeColor {
        GREEN(new Scalar(37,53,97), new Scalar(76,152,148)), // TODO:Calibrate color constants
        BLUE(new Scalar(103,23,92), new Scalar(179,126,136)),
        MAGENTA(new Scalar(289,74,90), new Scalar(309,94,110)),
        OTHER(new Scalar(0,0,0), new Scalar(0,0,0)); //leave OTHER as is
        public final Scalar lowHSV;
        public final Scalar highHSV;
        ConeColor(Scalar lowHSV, Scalar highHSV) { this.highHSV = highHSV; this.lowHSV = lowHSV;}

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

        Rect rectCrop = new Rect(480, 270, 960, 540); //TODO: Calibrate crop constants if necessary
        Mat crop = new Mat(mask, rectCrop);


        if (crop.empty()) {
            return input;
        }

        Mat threshMagenta = new Mat();
        Mat threshGreen = new Mat();
        Mat threshCyan = new Mat();

        Core.inRange(crop, ConeColor.MAGENTA.lowHSV, ConeColor.MAGENTA.highHSV, threshMagenta);
        Core.inRange(crop, ConeColor.GREEN.lowHSV, ConeColor.GREEN.highHSV, threshGreen);
        Core.inRange(crop, ConeColor.BLUE.lowHSV, ConeColor.BLUE.highHSV, threshCyan);

        if(Core.sumElems(threshMagenta).val[0] / rectCrop.area() / 255 > 0) {
            coneColor = ConeColor.MAGENTA;
        } else if(Core.sumElems(threshGreen).val[0] / rectCrop.area() / 255 > 0) {
            coneColor = ConeColor.GREEN;
        } else if(Core.sumElems(threshCyan).val[0] / rectCrop.area() / 255 > 0) {
            coneColor = ConeColor.BLUE;
        } else {
            coneColor = ConeColor.OTHER;
        }
        return crop;
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