package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
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
 * @see org.firstinspires.ftc.teamcode.Subsystems.ConeColorPipeline
 * @see <a href="https://github.com/OpenFTC/EasyOpenCV">EasyOpenCV</a>
 */
public class Vision extends Subsystem {
    public double distance;
    private static Vision theVision;
    public static final int CAMERA_WIDTH = 1920; // width of wanted camera resolution
    public static final int CAMERA_HEIGHT = 1080; // height of wanted camera resolution
    public static final int HORIZON = 100; // horizon value to tune
    public static final String WEBCAM_NAME =
            "Webcam 1"; // insert webcam name from configuration if using webcam
    public static final String VUFORIA_KEY =
            "ATDGULf/////AAABmRRGSyLSbUY4lPoqBYjklpYqC4y9J7bCk42kjgYS5KtgpKL8FbpEDQTovzZG8thxB01dClvthxkSuSyCkaZi+JiD5Pu0cMVre3gDwRvwRXA7V9kpoYyMIPMVX/yBTGaW8McUaK9UeQUaFSepsTcKjX/itMtcy7nl1k84JChE4i8whbinHWDpaNwb5qcJsXlQwJhE8JE7t8NMxMm31AgzqjVf/7HwprTRfrxjTjVx5v2rp+wgLeeLTE/xk1JnL3fZMG6yyxPHgokWlIYEBZ5gBX+WJfgA+TDsdSPY/MnBp5Z7QxQsO9WJA59o/UzyEo/9BkbvYJZfknZqeoZWrJoN9jk9sivFh0wIPsH+JjZNFsPw"; // TODO: Get new VUFORIA KEY
    // Since ImageTarget trackable use mm to specify their dimensions, we must use mm for all the
    // physical dimension.
    // Define constants
    private static final double mmPerInch = 25.4;
    // Constants for perimeter targets
    private static final double halfField = 72 * mmPerInch;
    private static final double quadField = 36 * mmPerInch;
    // Define where camera is in relation to center of robot in inches
    final double CAMERA_FORWARD_DISPLACEMENT = 6.0f * mmPerInch; // TODO: CALIBRATE WHEN ROBOT IS BUILT
    final double CAMERA_VERTICAL_DISPLACEMENT = 6.5f * mmPerInch;
    final double CAMERA_LEFT_DISPLACEMENT = -0.75f * mmPerInch;
    OpenGLMatrix robotFromCamera = null;
    private final HardwareMap hardwareMap;
    private final AllianceColor allianceColor;
    // Class Members
    private OpenGLMatrix lastLocation;
    private VuforiaLocalizer vuforia;

    private boolean targetVisible;
    private VectorF targetTranslation;
    private Orientation targetRotation;

    private ConeColorPipeline pipeline;
    private OpenCvCamera camera;

    private int[] viewportContainerIds;

    // Move stuff

    /**
     * Class instantiation
     *
     * @param telemetry   Telemetry
     * @param hardwareMap the hardware map
     * @param timer       how much time elapsed
     * @param allianceColor the alliance color
     */
    public Vision(
            Telemetry telemetry,
            HardwareMap hardwareMap,
            ElapsedTime timer,
            AllianceColor allianceColor) {
        super(telemetry, hardwareMap, timer);
        this.hardwareMap = hardwareMap;
        this.allianceColor = allianceColor;
        telemetry.addLine("Vision init started");
        telemetry.update();

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

        // Create a detection pipeline for detecting the position
        pipeline = new ConeColorPipeline(allianceColor, CAMERA_WIDTH, CAMERA_HEIGHT);
        camera.setPipeline(pipeline);

        // Create listeners for the camera
        camera.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() { // Listener for when the camera first starts
                        telemetry.addLine("Streaming");
                        camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                    }

                    @Override
                    public void onError(int errorCode) { // Listener to log if the camera stops abruptly
                        telemetry.addLine("Error Streaming, aborting");
                        telemetry.update();
                    }
                });
    }

    // Helper method to create matrix to identify locations
    public OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w) {
        return OpenGLMatrix.translation(x, y, z)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, u, v, w));
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

    public static Vision getVision() {
        return theVision;
    }
}