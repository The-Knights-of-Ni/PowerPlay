package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Control subsystem for controlling arms and claws
 */
public class Control extends Subsystem {

    // DC Motors
    private final DcMotorEx intake;
    private final DcMotorEx bucket;
    private final DcMotorEx slide;
    private final Servo lid;
    private final Servo markerSlide;
    private final Servo markerHook;

    private SlideState currentSlidePosition;

    // Servos
    private final CRServo duckWheel;

    // Sensors
//    private final DistanceSensor loadSensor;

    /**
     * Enums stores the semantic position of each motor along with motor constants in ticks when relevant
     */
    public enum PlacementLevel {
        TOP,
        MIDDLE,
        BOTTOM,
        NOT_FOUND
    }

    public enum BucketState {
        LEVEL(0, 0.3),
        RAISED(-90, 0.7);

        public final double power;
        public final int position;

        BucketState(int position, double power) {
            this.position = position;
            this.power = power;
        }
    }

    public enum SlideState {
        RETRACTED(0, 0.4),
        BOTTOM(465, 0.6),
        MIDDLE(720, 0.6),
        TOP(1375, 0.6);

        public final int position;
        public final double power;

        SlideState(int position, double power) {
            this.position = position;
            this.power = power;
        }
    }

    public enum LidPosition {
        CLOSED(0.8),
        DEPLOYED(0.65),
        OPEN(0.5);

        public final double position;

        LidPosition(double position) {
            this.position = position;
        }
    }

    public enum MarkerHookState {
        UP(0.20, 0.425), // Hook is perpendicular/level
        DOWN(0.45, 0); // Hook is angled downward

        public final double posHookServo;
        public final double posSlideServo;

        MarkerHookState(double posHookServo, double posSlideServo) {
            this.posHookServo = posHookServo;
            this.posSlideServo = posSlideServo;
        }
    }

    public Control(DcMotorEx intake, DcMotorEx bucket, DcMotorEx slide, CRServo duckWheel, Servo lid, Servo markerSlide, Servo markerHook,
                   BNO055IMU imu, DistanceSensor loadSensor,
                   Telemetry telemetry, HardwareMap hardwareMap, ElapsedTime timer) {
        super(telemetry, hardwareMap, timer);

        // Store device information locally
        this.intake = intake;
        this.bucket = bucket;
        this.slide = slide;
        this.duckWheel = duckWheel;
        this.lid = lid;
        this.markerSlide = markerSlide;
        this.markerHook = markerHook;
//        this.loadSensor = loadSensor;

        // Default for slide position
        this.currentSlidePosition = SlideState.RETRACTED;

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Set all DC motors to specified zero power behavior
     */
    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior mode) {
        intake.setZeroPowerBehavior(mode);
        bucket.setZeroPowerBehavior(mode);
        slide.setZeroPowerBehavior(mode);
    }

    /**
     * Toggle the intake and set its direction
     *
     * @param status    Indicates whether to turn the motor on (true) or off (false).
     * @param direction Specifies the direction to turn, where true/false corresponds to forward/reverse respectively.
     */
    public void setIntakeDirection(boolean status, boolean direction) {
        double power = status ? 0.7 : 0;

        if (direction) {
            intake.setPower(power);
        } else {
            intake.setPower(-power);
        }
    }

    /**
     * Set the position of the bucket
     *
     * @param bucketState   the bucket location. Must be either FLOOR, LEVEL, or RAISED.
     */
    public void setBucketState(BucketState bucketState) {
        bucket.setTargetPosition(bucketState.position);
        bucket.setPower(bucketState.power);
        bucket.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    /**
     * Set the position of the slide
     *
     * @param slideState the position to set the slide. Must be either RETRACTED, BOTTOM, MIDDLE, or TOP.
     */
    public void setSlide(SlideState slideState) {
        // Check for if programmers set position to the current position of slide smh
        if (slideState == currentSlidePosition) return;
        // Check if the target position or current position are retracted
        if (slideState == SlideState.RETRACTED || currentSlidePosition == SlideState.RETRACTED) {
            // Move to bottom first with programmable speed
            slide.setPower(currentSlidePosition.power);
            slide.setTargetPosition(SlideState.BOTTOM.position);
            slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        }
        // Move normally
        slide.setPower(slideState.power);
        slide.setTargetPosition(slideState.position);
        slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // Update current slide position
        currentSlidePosition = slideState;
    }

    /**
     * Set the position of the lid to drop the freight
     *
     * @param lidPosition   the position to set the lid. Must be either CLOSED, DEPLOYED, or OPEN.
     */
    public void setLidPosition(LidPosition lidPosition) {
        lid.setPosition(lidPosition.position);
    }

    /**
     * Start the duck wheel in order to spin the carousel
     *
     * @param direction the rotation direction. True/false corresponds to forward and backwards respectively.
     */
    public void startCarousel(boolean direction) {
        duckWheel.set(direction ? 1 : -1);
    }

    /**
     * Stop the carousel from rotating
     */
    public void stopCarousel() {
        duckWheel.set(0);
    }

    /**
     * Servo test/position calibrator method.
     * @param servo the servo to be tested/modified.
     * @param value the interval at which to adjust the position.
     */
    public void modifyServo(Servo servo, double value) {
        double currentValue = servo.getPosition();
        currentValue = currentValue + value;
        if (currentValue > 1.0) currentValue = 1.0;
        if (currentValue < 0.0) currentValue = 0.0;
        servo.setPosition(currentValue);
    }

    /**
     * Checks if the slide is retracted (helper method for higher level classes)
     */
    public boolean isSlideRetracted() {
        return Math.abs(slide.getCurrentPosition() - SlideState.RETRACTED.position) <= 3;
    }

    public void runMarkerHook(MarkerHookState state) {
        if(state == MarkerHookState.UP)
            markerHook.setPosition(state.posHookServo);
        markerSlide.setPosition(state.posSlideServo);

    }

    /**
     * Checks if the (initial) bucket is currently holding freight
     */
//    public boolean isLoaded() {
//        double BUCKET_WIDTH = 0; // TODO find out what this is
//        return loadSensor.getDistance(DistanceUnit.CM) < BUCKET_WIDTH;
//    }
}