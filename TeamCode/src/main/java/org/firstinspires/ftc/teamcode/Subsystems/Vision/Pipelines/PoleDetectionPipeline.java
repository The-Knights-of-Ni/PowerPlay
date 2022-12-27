package org.firstinspires.ftc.teamcode.Subsystems.Vision.Pipelines;

import android.util.Log;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.Vision;
import org.firstinspires.ftc.teamcode.Util.AllianceColor;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.CascadeClassifier;
import org.openftc.easyopencv.OpenCvPipeline;

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
public class PoleDetectionPipeline extends OpenCvPipeline {
    MatOfRect poles;

    /**
     * Class instantiation
     *
     * @see Robot
     * @see Telemetry
     * @see AllianceColor
     */
    public PoleDetectionPipeline() {
    }

    @Override
    public Mat processFrame(Mat input) {
        CascadeClassifier poleCascade = new CascadeClassifier();
        if (!poleCascade.load("")) { // TODO: Filename here
            Log.e("Pole Detection", "Error loading pole cascade");
        }
        Mat frameGray = new Mat();
        Imgproc.cvtColor(input, frameGray, Imgproc.COLOR_BGR2GRAY);
        Imgproc.equalizeHist(frameGray, frameGray);
        // -- Detect faces
        poles = new MatOfRect();
        poleCascade.detectMultiScale(frameGray, poles);
        return frameGray;
    }

    public List<Rect> getPoles() {
        return poles.toList();
    }
}