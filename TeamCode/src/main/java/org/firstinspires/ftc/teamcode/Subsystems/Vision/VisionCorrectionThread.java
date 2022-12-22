package org.firstinspires.ftc.teamcode.Subsystems.Vision;


import com.qualcomm.robotcore.hardware.HardwareMap;
import org.apache.commons.geometry.euclidean.twod.ConvexArea;

import org.apache.commons.geometry.euclidean.twod.Vector2D;
import org.apache.commons.geometry.euclidean.twod.path.LinePath;
import org.apache.commons.numbers.core.Precision;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.DriveControl.BoundingBox;

import org.firstinspires.ftc.teamcode.Util.Vector;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import android.util.Log;

import java.util.List;

import static org.firstinspires.ftc.teamcode.Subsystems.Drive.Drive.mmPerInch;
import static org.firstinspires.ftc.teamcode.Subsystems.Vision.Vision.*;


public class VisionCorrectionThread implements Runnable {
    private final VisionCorrectionThreadData vtd = VisionCorrectionThreadData.getVTD();
    private static final String TAG = "Vision Correction";
    private final WebcamName cameraName;
    private OpenCvCamera camera;
    HardwareMap hardwareMap;
    PoleDetectionPipeline pipeline;

    public VisionCorrectionThread(WebcamName cameraName, HardwareMap hardwareMap) {
        this.cameraName = cameraName;
        this.hardwareMap = hardwareMap;
        initDetectionPipeline();
    }

    private void initDetectionPipeline() {
        // Get the camera ID
        int cameraMonitorViewId =
                hardwareMap
                        .appContext
                        .getResources()
                        .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Obtain camera instance from ID
        camera =
                OpenCvCameraFactory.getInstance()
                        .createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);
        camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
        // Create a detection pipeline for detecting the position
        pipeline = new PoleDetectionPipeline();
        camera.setPipeline(pipeline);

        // Create listeners for the camera
        camera.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() { // Listener for when the camera first starts
                        Log.i(TAG, "Streaming");
                        camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                    }

                    @Override
                    public void onError(int errorCode) { // Listener to log if the camera stops abruptly
                        Log.e(TAG, "Error Streaming, aborting with error code: " + errorCode);
                    }
                });
    }

    @Override
    public void run() {
        List<Rect> poles = pipeline.getPoles();
        if (poles.size() != 1) {
            Log.e(TAG, "Pole Detection Failed");
        }
        Rect pole = poles.get(0);
        Vector offset = new Vector(pole.x + (pole.width/2), pole.y - (pole.height/2));
        vtd.setCorrectionVector(offset);
    }
}
