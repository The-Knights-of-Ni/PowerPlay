package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import android.util.Log;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.Pipelines.ConeColorPipeline;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.Pipelines.PoleDetectionPipeline;
import org.firstinspires.ftc.teamcode.Util.AllianceColor;
import org.firstinspires.ftc.teamcode.Util.Vector;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

/**
 * The Vision Subsystem
 *
 * @see ConeColorPipeline
 * @see <a href="https://github.com/OpenFTC/EasyOpenCV">EasyOpenCV</a>
 */
public class Vision extends Subsystem {
    public static final int CAMERA_WIDTH = 1920; // width of wanted camera resolution
    public static final int CAMERA_HEIGHT = 1080; // height of wanted camera resolution
    public static final String WEBCAM_NAME =
            "Webcam 1"; // insert webcam name from configuration if using webcam
    public static final String VUFORIA_KEY = "AdaaiLz/////AAABmY48m0KCh0LSrL2edcDLhdMoRzt30ceXdmqa4QAa4krhLO2RXrW8IHU" +
            "MXYoMmg9Jgb9rrD9KMlG/VpOmlMvKA5EEahmlY0Gf6AXH5PIoaVTIcaK6U4PtRBEKIUZ+x6qOhZsLW8j3nI3Rha1NQNxbwV5CAgzODPK" +
            "G8udq8VrXMnd2LBr46BQvxhZSmLLhJETGg6XKf563hbzGEg+6RO2oXwy10c0tax2vWgjFC4hRMiRf9HK8a7CYCnk7QG15syHv8ksuKBY" +
            "kG4YOIRfIMaxrq6B1KT709/PFFtdfBLYAYiKfdpu7Wmt6zGO1+dx003WBBDV80OjWRye05i0WcreTxmbGqbknMLxYm7ATIYNetDsG";
    private final HardwareMap hardwareMap;
    private final AllianceColor allianceColor;
    private final boolean visionCorrectionEnabled;

    private ConeColorPipeline coneColorPipeline;
    private PoleDetectionPipeline poleDetectionPipeline;
    private OpenCvCamera camera;


    /**
     * Class instantiation
     *
     * @param telemetry   Telemetry
     * @param hardwareMap the hardware map
     * @param allianceColor the alliance color
     */
    public Vision(
            Telemetry telemetry,
            HardwareMap hardwareMap,
            AllianceColor allianceColor,
            boolean visionCorrectionEnabled) {
        super(telemetry, "vision");
        this.hardwareMap = hardwareMap;
        this.allianceColor = allianceColor;
        this.visionCorrectionEnabled = visionCorrectionEnabled;
        // Create camera instances for the detection pipeline
        initDetectionPipeline();
        // Telemetry
        telemetry.addLine("Vision init complete");
        telemetry.update();
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
        coneColorPipeline = new ConeColorPipeline(allianceColor);
        poleDetectionPipeline = new PoleDetectionPipeline();
        camera.setPipeline(coneColorPipeline);

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


    public void stop() {
        // Stop streaming
        camera.stopStreaming();
        camera.closeCameraDevice();
    }

    /**
     * This method waits until the search for the marker is done, and then it return the marker
     * location. It waits until the marker is found, then it returns the marker location.
     *
     * @return Where the marker is
     */
    public ConeColorPipeline.ConeColor detectConeColor() {
        camera.setPipeline(coneColorPipeline);
        return coneColorPipeline.getConeColor();
    }

    public Vector getCorrectionVector() {
        if (visionCorrectionEnabled) {
            camera.setPipeline(coneColorPipeline);
            List<Rect> poles = poleDetectionPipeline.getPoles();
            if (poles.size() != 1) {
                Log.e("Vision Correction", "Pole Detection Failed");
                return Vector.ZERO;
            }
            Rect pole = poles.get(0);
            return new Vector(pole.x + (pole.width / 2), pole.y - (pole.height / 2));
        }
        return Vector.ZERO;
    }
}
