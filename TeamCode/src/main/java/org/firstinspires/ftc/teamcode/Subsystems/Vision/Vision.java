package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import android.util.Log;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Util.AllianceColor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

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
    // Since ImageTarget trackable use mm to specify their dimensions, we must use mm for all the
    // physical dimension.
    // Define constants
    private static final double mmPerInch = 25.4;
    // Define where camera is in relation to center of robot in inches
    final static double CAMERA_FORWARD_DISPLACEMENT = 6.0f * mmPerInch; // TODO: CALIBRATE WHEN ROBOT IS BUILT
    final static double CAMERA_VERTICAL_DISPLACEMENT = 6.5f * mmPerInch;
    final static double CAMERA_LEFT_DISPLACEMENT = -0.75f * mmPerInch;
    OpenGLMatrix robotFromCamera = null;
    private final HardwareMap hardwareMap;
    private final AllianceColor allianceColor;
    // Class Members
    private OpenGLMatrix lastLocation;
    private VuforiaLocalizer vuforia;

    private VectorF targetTranslation;
    private Orientation targetRotation;

    private ConeColorPipeline pipeline;
    private OpenCvCamera camera;

    private int[] viewportContainerIds;

    VisionCorrectionThread visionCorrectionThread;

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

        // Telemetry
        telemetry.addLine("Vision init complete");
        telemetry.update();
        if (visionCorrectionEnabled) {
            WebcamName webcamName = hardwareMap.get(WebcamName.class, WEBCAM_NAME);
            visionCorrectionThread = new VisionCorrectionThread(webcamName);
        }
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
        pipeline = new ConeColorPipeline(CAMERA_WIDTH, CAMERA_HEIGHT);
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

    // Helper method to create matrix to identify locations
    public OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w) {
        return OpenGLMatrix.translation(x, y, z)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, u, v, w));
    }

    public void start() {
        // Create camera instances for the detection pipeline
        initDetectionPipeline();
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
        // Return the marker location
        return pipeline.getConeColor();
    }
}
