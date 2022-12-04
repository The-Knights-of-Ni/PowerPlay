package org.firstinspires.ftc.teamcode.Subsystems.Drive;

import android.util.Log;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.apache.commons.geometry.euclidean.twod.ConvexArea;
import org.apache.commons.geometry.euclidean.twod.Vector2D;
import org.apache.commons.geometry.euclidean.twod.path.LinePath;
import org.apache.commons.numbers.core.Precision;
import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DriveControl.BoundingBox;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionCorrectionThreadData;
import org.firstinspires.ftc.teamcode.Subsystems.Web.WebThreadData;
import org.firstinspires.ftc.teamcode.Util.Coordinate;
import org.firstinspires.ftc.teamcode.Util.Vector;

import java.util.Arrays;
import java.util.Locale;

/**
 * Mecanum drivetrain subsystem
 */
public class Drive extends Subsystem {
    /**
     * The number of millimeters per a count in odometry
     */
    private static final double ODOMETRY_mm_PER_COUNT = 38.85 * 3.14159265 / 8192.0;
    private static final double ODOMETRY_RADIUS_X = 201.0;
    private static final double ODOMETRY_RADIUS_Y = 178.0;
    // DO WITH ENCODERS
    private static final double DRIVE_GEAR_REDUCTION = 1.0; // This is < 1.0 if geared UP
    private static final double TICKS_PER_MOTOR_REV_20 = 537.6; // AM Orbital 20 motor
    private static final double RPM_MAX_NEVERREST_20 = 340;
    private static final double ANGULAR_V_MAX_NEVERREST_20 =
            (TICKS_PER_MOTOR_REV_20 * RPM_MAX_NEVERREST_20) / 60.0;
    // NEW Chassis
    private static final double MOTOR_TICK_PER_REV_YELLOW_JACKET_312 = 537.6;
    private static final double GOBUILDA_MECANUM_DIAMETER_MM = 96.0;
    private static final double COUNTS_PER_MM =
            (MOTOR_TICK_PER_REV_YELLOW_JACKET_312 * DRIVE_GEAR_REDUCTION)
                    / (GOBUILDA_MECANUM_DIAMETER_MM * Math.PI);
    private static final double WHEEL_DIAMETER_INCHES = 100.0 / 25.4; // For calculating circumference
    /**
     * Wheel Diameter MM
     */
    private static final double WHEEL_DIAMETER_MM = 100.0;
    private static final double COUNTS_PER_INCH =
            (TICKS_PER_MOTOR_REV_20 * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double COUNTS_CORRECTION_X = 1.150;
    private static final double COUNTS_CORRECTION_Y = 0.9918;
    private static final double COUNTS_PER_DEGREE = 10; // 900 ticks per 90 degrees
    /**
     * Default drive speeds
     */
    private static final double DRIVE_SPEED = 0.40;
    private static final double DRIVE_SPEED_X = 0.35;
    private static final double DRIVE_SPEED_Y = 0.40;
    /**
     * Default turn speed
     */
    private static final double TURN_SPEED = 0.40;
    /**
     * Number of millimeters per an Inch
     */
    public static final double mmPerInch = 25.4;
    private static final double motorKp = 0.015;
    private static final double motorKi = 0.02;
    private static final double motorKd = 0.0003;
    private static final double motorRampTime = 0.3;
    private static final double DEGREES_TO_RADIANS = Math.PI/180;
    /**
     * DC Motor front left
     */
    public final DcMotorEx frontLeft;
    /**
     * DC Motor front right
     */
    public final DcMotorEx frontRight;
    /**
     * DC Motor rear left
     */
    public final DcMotorEx rearLeft;
    /**
     * DC Motor rear right
     */
    public final DcMotorEx rearRight;
    private final ElapsedTime timer;
    // use motor encoder for odometry
    /**
     * Odometry Left
     */
    public DcMotorEx odL;
    /**
     * Odometry Back
     */
    public DcMotorEx odB;
    /**
     * Odometry Right
     */
    public DcMotorEx odR;
    /**
     * Current Robot x position in millimeters
     */
    private double robotCurrentPosX;
    /**
     * Current Robot y position in millimeters
     */
    private double robotCurrentPosY;
    /**
     * Current Robot angle
     */
    private double robotCurrentAngle;
    private int encoderOffsetFL = 0;
    private int encoderOffsetFR = 0;
    private int encoderOffsetRL = 0;
    private int encoderOffsetRR = 0;
    private int odometryCountOffsetL = 0;
    private int odometryCountOffsetR = 0;
    private int odometryCountOffsetB = 0;
    private int odometryCountL = 0;
    private int odometryCountR = 0;
    private int odometryCountB = 0;

    private long startTime;

    private VisionCorrectionThreadData vtd;
    private WebThreadData wtd;

    /**
     * Initializes the drive subsystem
     *
     * @param frontLeft   The front left motor in the drive train
     * @param frontRight  The front right motor
     * @param rearLeft    The rear left motor
     * @param rearRight   The rear right motor
     * @param telemetry   The telemetry
     * @param elapsedTime       The timer for the elapsed time
     */
    public Drive(DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx rearLeft, DcMotorEx rearRight, Telemetry telemetry, ElapsedTime elapsedTime, boolean updateVTD, boolean updateWeb) {
        super(telemetry, "drive");
        this.timer = elapsedTime;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.rearLeft = rearLeft;
        this.rearRight = rearRight;
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotCurrentPosX = 0;
        robotCurrentPosY = 0;
//        if (updateVTD)
//        vtd = VisionCorrectionThreadData.getVTD();
//        if (updateWeb)
//        wtd = WebThreadData.getWtd();
    }

    /**
     * Logs the current robot position via Log.i
     * @see Log
     */
    private void logMovement() {
        try {
            Log.i(TAG, "Robot X: " + robotCurrentPosX + " Robot Y: " + robotCurrentPosY + " Robot Turn: " +
                    robotCurrentAngle);
        }
        catch (RuntimeException ignored) {

        }
    }

    /**
     * Gets the left odometry
     *
     * @return The left odometry
     */
    private int getOdometryCountL() {
        odometryCountL = -odL.getCurrentPosition() - odometryCountOffsetL;
        return odometryCountL;
    }

    /**
     * Gets the back odometry
     *
     * @return The back odometry
     */
    private int getOdometryCountB() {
        odometryCountB = odB.getCurrentPosition() - odometryCountOffsetB;
        return odometryCountB;
    }

    /**
     * Gets the right odometry
     *
     * @return The right odometry
     */
    private int getOdometryCountR() {
        odometryCountR = odR.getCurrentPosition() - odometryCountOffsetR;
        return odometryCountR;
    }

    /**
     * Reset the odometry.
     */
    private void resetOdometry() {
        odometryCountOffsetL = -odL.getCurrentPosition();
        odometryCountOffsetB = odB.getCurrentPosition();
        odometryCountOffsetR = odR.getCurrentPosition();
    }

    /**
     * Updates the odometry by getting the left back and right odometry.
     */
    private void updateOdometry() {
        getOdometryCountL();
        getOdometryCountB();
        getOdometryCountR();
    }

    public double getAngularVMaxNeverrest20() {
        return ANGULAR_V_MAX_NEVERREST_20;
    }

    /**
     * Stops all drive motors by setting the powers to 0
     */
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
    }

    /**
     * Stops the motors only if they are not busy.
     */
    public void checkAndStopMotors() {
        if (!frontLeft.isBusy()) {
            frontLeft.setPower(0);
        }
        if (!frontRight.isBusy()) {
            frontRight.setPower(0);
        }
        if (!rearLeft.isBusy()) {
            rearLeft.setPower(0);
        }
        if (!rearRight.isBusy()) {
            rearRight.setPower(0);
        }
    }

    /**
     * Sets all drive motors to specified run mode
     *
     * @param mode the specified mode
     */
    public void setRunMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        rearLeft.setMode(mode);
        rearRight.setMode(mode);
    }

    /**
     * Initialize MaxVelocity of drive motors
     */
    public void initMaxVelocity() {
        frontLeft.setVelocity(ANGULAR_V_MAX_NEVERREST_20);
        frontRight.setVelocity(ANGULAR_V_MAX_NEVERREST_20);
        rearLeft.setVelocity(ANGULAR_V_MAX_NEVERREST_20);
        rearRight.setVelocity(ANGULAR_V_MAX_NEVERREST_20);
    }

    /**
     * Sets all drive motors to specified zero power behavior
     */
    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior mode) {
        frontLeft.setZeroPowerBehavior(mode);
        frontRight.setZeroPowerBehavior(mode);
        rearLeft.setZeroPowerBehavior(mode);
        rearRight.setZeroPowerBehavior(mode);
    }

    /**
     * Calculates the motor powers when given the position o the left and right sticks
     *
     * @param leftStickX  left joystick x position
     * @param leftStickY  left joystick y position
     * @param rightStickX right joystick x position for turning
     * @return A list with the motor powers
     */
    public double[] calcMotorPowers(double leftStickX, double leftStickY, double rightStickX) { // split direction from controller into wheel powers
        // Get displacement of joystick for the power
        double r = Math.hypot(leftStickX, leftStickY);
        // Get angle of the joystick from the positive x-axis shifted by PI/4 clockwise
        // PI/4 shift is for strafing
        double robotAngle = Math.atan2(leftStickY, leftStickX) - Math.PI / 4;
        // The second terms of each of these statements is for turning in place
        // The other stuff is funky strafing + movement
        double lrPower = r * Math.sin(robotAngle) + rightStickX;
        double lfPower = r * Math.cos(robotAngle) + rightStickX;
        double rrPower = r * Math.cos(robotAngle) - rightStickX;
        double rfPower = r * Math.sin(robotAngle) - rightStickX;
        // Return the calculated powers
        return new double[]{lfPower, rfPower, lrPower, rrPower};
    }

    /**
     * Sets the drive power to every motor
     *
     * @param power the power to set the motors to
     */
    public void setDrivePower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        rearLeft.setPower(power);
        rearRight.setPower(power);
    }

    /**
     * Sets the drive power of each motor individually.
     *
     * @param powers the powers to set each of the motors to
     */
    public void setDrivePowers(double[] powers) {
        frontLeft.setPower(powers[0]);
        frontRight.setPower(powers[1]);
        rearLeft.setPower(powers[2]);
        rearRight.setPower(powers[3]);
    }

    /**
     * Turns the robot by the specified angle, ticks and angles are equivalent. Use this method to turn the robot instead
     *
     * @param angle The angle to turn by. Positive corresponds to counterclockwise
     */
    public void turnByAngle(double angle) {
        moveVector(new Vector(0, 0), angle);
        robotCurrentAngle += angle;
        telemetry.addData("turnRobot", "turn to %7.2f degrees", robotCurrentAngle);
        telemetry.update();
    }

    /**
     * Corrects the angle of the robot, to prevent the robot from being turned in the wrong position.
     */
    private void correctAngleOdometry() {
        updateOdometry();
        telemetry.addData(
                "odometry",
                " L %7.2f R %7.2f B %7.2f",
                odometryCountL * ODOMETRY_mm_PER_COUNT,
                odometryCountR * ODOMETRY_mm_PER_COUNT,
                odometryCountB * ODOMETRY_mm_PER_COUNT);
        telemetry.update();
        double angleError =
                (((double) odometryCountR) - ((double) odometryCountL))
                        * 0.5
                        * ODOMETRY_mm_PER_COUNT
                        * (180.0 / 3.14159265)
                        / ODOMETRY_RADIUS_X;
        turnByAngle(-angleError);
        updateOdometry();
        Log.d("odometry", "correction angle " + (-angleError));
        Log.v("odometry", " L " + odometryCountL * ODOMETRY_mm_PER_COUNT + " R " +
                odometryCountR * ODOMETRY_mm_PER_COUNT + " B " + odometryCountB * ODOMETRY_mm_PER_COUNT);
    }

    /**
     * Moves the robot forward by the specified distance with the default speed with odometry.
     *
     * @param distance The distance to move forward by, in millimeters, use the mmPerInch constant if you want to use
     *                 inches.
     */
    public void moveForwardOdometry(double distance) {
        moveForwardOdometry(distance, DRIVE_SPEED_Y);
    }

    /**
     * Moves the robot forward by the specified distance with the specified speed with odometry.
     *
     * @param distance   The distance to move forward by
     * @param motorSpeed The speed, a value between 0 and 1
     */
    public void moveForwardOdometry(double distance, double motorSpeed) {
        resetOdometry();
        moveForward(distance, motorSpeed);
        correctAngleOdometry();
        if (odometryCountB * ODOMETRY_mm_PER_COUNT > 25.0) {
            moveLeft(odometryCountB * ODOMETRY_mm_PER_COUNT);
        }
        if (odometryCountB * ODOMETRY_mm_PER_COUNT < -25.0) {
            moveRight(-odometryCountB * ODOMETRY_mm_PER_COUNT);
        }
    }

    /**
     * Moves the robot forward by the specified distance with the default speed.
     *
     * @param distance The distance to move forward by, in millimeters, use the mmPerInch constant if you want to use
     *                 inches.
     */
    public void moveForward(double distance) {
        moveForward(distance, DRIVE_SPEED_Y);
    }

    /**
     * Moves the robot forward by the specified distance with the specified speed.
     *
     * @param distance   The distance to move forward by
     * @param motorSpeed The speed, a value between 0 and 1
     */
    public void moveForward(double distance, double motorSpeed) {
        moveVector(new Vector(0, distance), 0, motorSpeed);
        logMovement();
    }

    /**
     * Moves the robot backwards by the specified distance with the default speed with odometry.
     *
     * @param distance The distance to move backward by, in millimeters, use the mmPerInch constant if you want to use
     *                 inches.
     */
    public void moveBackwardOdometry(double distance) {
        moveBackwardOdometry(distance, DRIVE_SPEED_Y);
    }

    /**
     * Moves the robot backwards by the specified distance with the specified speed with odometry.
     *
     * @param distance   The distance to move forward by
     * @param motorSpeed The speed, a value between 0 and 1
     */
    public void moveBackwardOdometry(double distance, double motorSpeed) {
        resetOdometry();
        moveBackward(distance, motorSpeed);
        correctAngleOdometry();
        if (odometryCountB * ODOMETRY_mm_PER_COUNT > 25.0) {
            moveLeft(odometryCountB * ODOMETRY_mm_PER_COUNT);
        }
        if (odometryCountB * ODOMETRY_mm_PER_COUNT < -25.0) {
            moveRight(-odometryCountB * ODOMETRY_mm_PER_COUNT);
        }
    }

    /**
     * Moves the robot backwards by the specified distance with the default speed.
     *
     * @param distance The distance to move backward by, in millimeters, use the mmPerInch constant if you want to use
     *                 inches.
     */
    public void moveBackward(double distance) {
        moveBackward(distance, DRIVE_SPEED_Y);
    }

    /**
     * Moves the robot backwards by the specified distance with the specified speed.
     *
     * @param distance   The distance to move forward by
     * @param motorSpeed The speed, a value between 0 and 1
     */
    public void moveBackward(double distance, double motorSpeed) {
        moveVector(new Vector(0, -distance), 0, motorSpeed);
        logMovement();
    }

    /**
     * Strafes the robot left by the specified distance with the default speed with odometry.
     *
     * @param distance The distance to strafe left by, in millimeters, use the mmPerInch constant if you want to use
     *                 inches.
     */
    public void moveLeftOdometry(double distance) {
        moveLeftOdometry(distance, DRIVE_SPEED_X);
    }

    /**
     * Strafes the robot left by the specified distance with the specified speed with odometry.
     *
     * @param distance   The distance to strafe left by
     * @param motorSpeed The speed, a value between 0 and 1
     */
    public void moveLeftOdometry(double distance, double motorSpeed) {
        resetOdometry();
        moveLeft(distance, motorSpeed);
        correctAngleOdometry();
        double offsetY = (((double) odometryCountR) + ((double) odometryCountL)) * 0.5;
        if (offsetY * ODOMETRY_mm_PER_COUNT > 25.0) {
            moveBackward(offsetY * ODOMETRY_mm_PER_COUNT);
        }
        if (offsetY * ODOMETRY_mm_PER_COUNT < -25.0) {
            moveForward(-offsetY * ODOMETRY_mm_PER_COUNT);
        }
    }

    /**
     * Strafes the robot left by the specified distance with the default speed.
     *
     * @param distance The distance to strafe left by, in millimeters, use the mmPerInch constant if you want to use
     *                 inches.
     */
    public void moveLeft(double distance) {
        moveLeft(distance, DRIVE_SPEED_X);
    }

    /**
     * Strafes the robot left by the specified distance with the specified speed.
     *
     * @param distance   The distance to strafe left by
     * @param motorSpeed The speed, a value between 0 and 1
     */
    public void moveLeft(double distance, double motorSpeed) {
        moveVector(new Vector(-distance, 0), 0, motorSpeed);
        logMovement();
    }

    /**
     * Strafes the robot right by the specified distance with the default speed with odometry.
     *
     * @param distance The distance to strafe right by, in millimeters, use the mmPerInch constant if you want to use
     *                 inches.
     */
    public void moveRightOdometry(double distance) {
        moveRightOdometry(distance, DRIVE_SPEED_X);
    }

    /**
     * Strafes the robot right by the specified distance with the specified speed with odometry.
     *
     * @param distance   The distance to strafe right by
     * @param motorSpeed The speed, a value between 0 and 1
     */
    public void moveRightOdometry(double distance, double motorSpeed) {
        resetOdometry();
        moveRight(distance, motorSpeed);
        correctAngleOdometry();
        double offsetY = (((double) odometryCountR) + ((double) odometryCountL)) * 0.5;
        if (offsetY * ODOMETRY_mm_PER_COUNT > 25.0) {
            moveBackward(offsetY * ODOMETRY_mm_PER_COUNT);
        }
        if (offsetY * ODOMETRY_mm_PER_COUNT < -25.0) {
            moveForward(-offsetY * ODOMETRY_mm_PER_COUNT);
        }
    }

    /**
     * Strafes the robot right by the specified distance with the default speed.
     *
     * @param distance The distance to strafe right by, in millimeters, use the mmPerInch constant if you want to use
     *                 inches.
     */
    public void moveRight(double distance) {
        moveRight(distance, DRIVE_SPEED_X);
    }

    /**
     * Strafes the robot right by the specified distance with the specified speed.
     *
     * @param distance   The distance to strafe right by
     * @param motorSpeed The speed, a value between 0 and 1
     */
    public void moveRight(double distance, double motorSpeed) {
        moveVector(new Vector(distance, 0), 0, motorSpeed);
        logMovement();
    }

    /**
     * Print the motor PID coefficients to telemetry
     */
    public void printMotorPIDCoefficients() {
        PIDFCoefficients pidfCoefficients;
        pidfCoefficients = getMotorPIDCoefficients(frontLeft, DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData(
                "Front Left ",
                "P: %.2f I: %.2f D: %.2f F: %.2f A: %s",
                pidfCoefficients.p,
                pidfCoefficients.i,
                pidfCoefficients.d,
                pidfCoefficients.f,
                pidfCoefficients.algorithm.toString());
        pidfCoefficients = getMotorPIDCoefficients(frontRight, DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData(
                "Front Right",
                "P: %.2f I: %.2f D: %.2f F: %.2f A: %s",
                pidfCoefficients.p,
                pidfCoefficients.i,
                pidfCoefficients.d,
                pidfCoefficients.f,
                pidfCoefficients.algorithm.toString());
        pidfCoefficients = getMotorPIDCoefficients(rearLeft, DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData(
                "Rear Left  ",
                "P: %.2f I: %.2f D: %.2f F: %.2f A: %s",
                pidfCoefficients.p,
                pidfCoefficients.i,
                pidfCoefficients.d,
                pidfCoefficients.f,
                pidfCoefficients.algorithm.toString());
        pidfCoefficients = getMotorPIDCoefficients(rearRight, DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData(
                "Rear Right ",
                "P: %.2f I: %.2f D: %.2f F: %.2f A: %s",
                pidfCoefficients.p,
                pidfCoefficients.i,
                pidfCoefficients.d,
                pidfCoefficients.f,
                pidfCoefficients.algorithm.toString());
        telemetry.update();
    }

    public void setMotorKp(double motorKPFL, double motorKPFR, double motorKPRL, double motorKPRR) {
        frontLeft.setPositionPIDFCoefficients(motorKPFL);
        frontRight.setPositionPIDFCoefficients(motorKPFR);
        rearLeft.setPositionPIDFCoefficients(motorKPRL);
        rearRight.setPositionPIDFCoefficients(motorKPRR);
    }

    /**
     * Sets the motor pid coefficients
     *
     * @param Kp
     * @param Ki
     * @param Kd
     * @param Kf
     */
    public void setMotorPID(double Kp, double Ki, double Kd, double Kf) {
        PIDFCoefficients pidFCoefficients = new PIDFCoefficients();
        pidFCoefficients.p = Kp;
        pidFCoefficients.i = Ki;
        pidFCoefficients.d = Kd;
        pidFCoefficients.f = Kf;
        pidFCoefficients.algorithm = MotorControlAlgorithm.PIDF;
        setMotorPIDCoefficients(frontLeft, DcMotor.RunMode.RUN_TO_POSITION, pidFCoefficients);
        setMotorPIDCoefficients(frontRight, DcMotor.RunMode.RUN_TO_POSITION, pidFCoefficients);
        setMotorPIDCoefficients(rearLeft, DcMotor.RunMode.RUN_TO_POSITION, pidFCoefficients);
        setMotorPIDCoefficients(rearRight, DcMotor.RunMode.RUN_TO_POSITION, pidFCoefficients);
    }

    /**
     * Gets the Motor PID Coefficients
     *
     * @param motor The motor to calculate the PID Coefficients
     * @param mode  The Run Mode
     * @return The PIDFCoefficients
     */
    public PIDFCoefficients getMotorPIDCoefficients(DcMotorEx motor, DcMotor.RunMode mode) {
        // get a reference to the motor controller and cast it as an extended functionality controller.
        // we assume it's a REV Robotics Expansion Hub (which supports the extended controller
        // functions).
        DcMotorControllerEx motorControllerEx = (DcMotorControllerEx) motor.getController();

        // get the port number of our configured motor.
        int motorIndex = motor.getPortNumber();

        // get the PID coefficients for the specific motor mode.

        return motorControllerEx.getPIDFCoefficients(motorIndex, mode);
    }

    /**
     * Sets the pid coefficients
     *
     * @param motor            The motor
     * @param mode             The run mode
     * @param pidfCoefficients The pid coefficients.
     */
    public void setMotorPIDCoefficients(
            DcMotorEx motor, DcMotor.RunMode mode, PIDFCoefficients pidfCoefficients) {
        // get a reference to the motor controller and cast it as an extended functionality controller.
        // we assume it's a REV Robotics Expansion Hub (which supports the extended controller
        // functions).
        DcMotorControllerEx motorControllerEx = (DcMotorControllerEx) motor.getController();

        // get the port number of our configured motor.
        int motorIndex = motor.getPortNumber();

        // get the PID coefficients for the specific motor mode.
        motorControllerEx.setPIDFCoefficients(motorIndex, mode, pidfCoefficients);
    }

    /**
     * Logs the drive encoder results to logcat
     */
    public void logDriveEncoders() {
        int currentCountFL = frontLeft.getCurrentPosition();
        double currentTimeFL = ((double) (timer.nanoseconds() - startTime)) * 1.0e-6;
        int currentCountFR = frontRight.getCurrentPosition();
        double currentTimeFR = ((double) (timer.nanoseconds() - startTime)) * 1.0e-6;
        int currentCountRL = rearLeft.getCurrentPosition();
        double currentTimeRL = ((double) (timer.nanoseconds() - startTime)) * 1.0e-6;
        int currentCountRR = rearRight.getCurrentPosition();
        double currentTimeRR = ((double) (timer.nanoseconds() - startTime)) * 1.0e-6;
        String output =
                String.format(
                        Locale.US,
                        "FL %.3f, %d, FR %.3f %d, RL %.3f %d, RR %.3f %d",
                        currentTimeFL,
                        currentCountFL,
                        currentTimeFR,
                        currentCountFR,
                        currentTimeRL,
                        currentCountRL,
                        currentTimeRR,
                        currentCountRR);
        Log.d(TAG, output);
    }


    /**
     * PID motor control program to ensure all four motors are synchronized
     *
     * @param tickCount: absolute value of target tick count of each motor
     * @param motorPowers: peak speed of motor rotation in tick per second
     * @param rampTime:  motor speed ramp uptime/downtime in sec (the amount of time it takes for the motor to reach the desired speed)
     * @param Kp:        coefficient Kp
     * @param Ki:        coefficient Ki
     * @param Kd:        coefficient Kd
     */
    public void allMotorPIDControl(
            int[] tickCount,
            double[] motorPowers,
            double rampTime,
            double Kp,
            double Ki,
            double Kd) {
        stop();
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boolean isMotorFLDone = false;
        boolean isMotorFRDone = false;
        boolean isMotorRLDone = false;
        boolean isMotorRRDone = false;
        boolean isMotorFLNotMoving = false;
        boolean isMotorFRNotMoving = false;
        boolean isMotorRLNotMoving = false;
        boolean isMotorRRNotMoving = false;
        boolean isTimeOutStarted = false;
        boolean isTimeOutExceeded = false;
        double timeOutPeriod =
                0.1; // program will time out if the motors got stuck for more than 0.1 second
        double timeOutStartedTime = 0.0;
        int timeOutThreshold =
                3; // motor is considered to be stuck if the motor count does not change more than 2 ticks
        double acculErrorFL = 0.0;
        double acculErrorFR = 0.0;
        double acculErrorRL = 0.0;
        double acculErrorRR = 0.0;
        double prevErrorFL = 0.0;
        double prevErrorFR = 0.0;
        double prevErrorRL = 0.0;
        double prevErrorRR = 0.0;
        double prevTimeFL = 0.0;
        double prevTimeFR = 0.0;
        double prevTimeRL = 0.0;
        double prevTimeRR = 0.0;
        boolean initialized = false; // disable Ki and Kd terms in first iteration
        int currentCountFL = 0;
        int currentCountFR = 0;
        int currentCountRL = 0;
        int currentCountRR = 0;
        int targetCountFL = 0;
        int targetCountFR = 0;
        int targetCountRL = 0;
        int targetCountRR = 0;
        int prevCountFL = 0;
        int prevCountFR = 0;
        int prevCountRL = 0;
        int prevCountRR = 0;
        double currentError = 0.0;
        double currentTargetSpeedFL, currentTargetSpeedFR, currentTargetSpeedRL, currentTargetSpeedRR;
        double currentPower = 0.0;
        double alpha = 0.95;
        double startTime = ((double) timer.nanoseconds()) * 1.0e-9;
        double currentTime = 0.0;
        double errorSlope = 0.0;
        // PID loop
        while (((!isMotorFLDone) || (!isMotorFRDone) || (!isMotorRLDone) || (!isMotorRRDone))
                && (!isTimeOutExceeded)) {
            // Front left motor control
            if (!isMotorFLDone) {
                currentCountFL = frontLeft.getCurrentPosition(); // get current motor tick

                currentTime = ((double) timer.nanoseconds()) * 1.0e-9 - startTime; // get current time
                targetCountFL = getTargetTickCount(
                                tickCount[0],
                                motorPowers[0],
                                rampTime,
                                currentTime); // get integrated target tick on the speed profile
                currentTargetSpeedFL =
                        getTargetSpeed(
                                tickCount[0],
                                motorPowers[0],
                                rampTime,
                                currentTime); // get the target speed on the speed profile
                if (initialized) { // check if the motor is rotating first time runing
                    // Check if moving
                    isMotorFLNotMoving = Math.abs(currentCountFL - prevCountFL) < timeOutThreshold;
                }
                if (tickCount[0] == 0 || currentCountFL * (tickCount[0] / Math.abs(tickCount[0])) >= Math.abs(tickCount[0])) {
                    isMotorFLDone = true;
                    isMotorFLNotMoving = true;
                    frontLeft.setPower(0.0);

                    prevErrorFL = 0;
                    prevTimeFL = 0;
                    prevCountFL = 0;
                } else {
                    currentError = targetCountFL - currentCountFL;
                    if (initialized) { // after the first point, the previous data is valid
                        // Calculate le I term
                        acculErrorFL =
                                acculErrorFL * alpha
                                        + currentError * (currentTime - prevTimeFL); // integrate error
                        // Calculate le D term
                            errorSlope = (currentError - prevErrorFL) / (currentTime - prevTimeFL); // error slope
                        // Calculate le P + I + D
                        currentPower = getCurrentPower(currentTargetSpeedFL, Kp, Ki, Kd, acculErrorFL, currentError, errorSlope); // apply PID correction
                    } else { // at the first point, use Kp only at the first point when we can't do I or D
                        currentPower = motorPowers[0] + currentError * Kp;
                    }
                    // Cap the powers at 0 and 1
                    if (currentPower > Math.abs(motorPowers[0])) currentPower = Math.abs(motorPowers[0]);
                    if (currentPower < -Math.abs(motorPowers[0])) currentPower = -Math.abs(motorPowers[0]);
                    // Set power of motor
                    frontLeft.setPower(currentPower);

                    // Track previous errors
                    prevErrorFL = currentError;
                    prevTimeFL = currentTime;
                    prevCountFL = currentCountFL;
                }
            }
            // Front right motor control
            if (!isMotorFRDone) {
                currentCountFR = frontRight.getCurrentPosition();
                currentTime = ((double) timer.nanoseconds()) * 1.0e-9 - startTime;
                targetCountFR = getTargetTickCount(tickCount[1], motorPowers[1], rampTime, currentTime);
                currentTargetSpeedFR = getTargetSpeed(tickCount[1], motorPowers[1], rampTime, currentTime);
                if (initialized) { // check if the motor is rotating
                    isMotorFRNotMoving = Math.abs(currentCountFR - prevCountFR) < timeOutThreshold;
                }
                if (tickCount[1] == 0 || currentCountFL * (tickCount[1] / Math.abs(tickCount[1])) >= Math.abs(tickCount[1])) {
                    isMotorFRDone = true;
                    isMotorFRNotMoving = true;
                    frontRight.setPower(0.0);
                    prevErrorFR = 0;
                    prevTimeFR = 0;
                    prevCountFR = 0;
                } else {
                    currentError = targetCountFR - currentCountFR;
                    if (initialized) { // after the first point, the previous data is valid
                        // Calculate le I
                        acculErrorFR =
                                acculErrorFR * alpha
                                        + currentError * (currentTime - prevTimeFR); // integrate error
                        // Calculate le D
                        errorSlope = (currentError - prevErrorFR) / (currentTime - prevTimeFR); // error slope
                        // Calculate le P + I + D
                        currentPower =
                                getCurrentPower(currentTargetSpeedFR, Kp, Ki, Kd, acculErrorFR, currentError, errorSlope); // apply PID correction
                    } else { // at the first point, use Kp only
                        // Calculate only P at the first run
                        currentPower = motorPowers[1] + currentError * Kp;
                    }
                    // Cap the powers at 0 and 1
                    if (currentPower > Math.abs(motorPowers[1])) currentPower = Math.abs(motorPowers[1]);
                    if (currentPower < -Math.abs(motorPowers[1])) currentPower = -Math.abs(motorPowers[1]);
                    // Set power of motor
                    frontRight.setPower(currentPower);

                    // Track previous errors
                    prevErrorFR = currentError;
                    prevTimeFR = currentTime;
                    prevCountFR = currentCountFR;
                }
            }
            // Rear left motor control
            if (!isMotorRLDone) {
                currentCountRL = rearLeft.getCurrentPosition(); // Get current motor tick
                currentTime = ((double) timer.nanoseconds()) * 1.0e-9 - startTime; // Get current time
                targetCountRL = getTargetTickCount(tickCount[2], motorPowers[2], rampTime, currentTime); // Get the target count
                currentTargetSpeedRL = getTargetSpeed(tickCount[2], motorPowers[2], rampTime, currentTime);
                if (initialized) { // check if the motor is rotating
                    isMotorRLNotMoving = Math.abs(currentCountRL - prevCountRL) < timeOutThreshold;
                }
                if (tickCount[2] == 0 || currentCountFL * (tickCount[2] / Math.abs(tickCount[2])) >= Math.abs(tickCount[2])) {
                    isMotorRLDone = true;
                    isMotorRLNotMoving = true;
                    rearLeft.setPower(0.0);

                    prevErrorRL = 0;
                    prevTimeRL = 0;
                    prevCountRL = 0;
                } else {
                    currentError = targetCountRL - currentCountRL;
                    if (initialized) { // after the first point, the previous data is valid
                        acculErrorRL =
                                acculErrorRL * alpha
                                        + currentError * (currentTime - prevTimeRL); // integrate error
                        errorSlope = (currentError - prevErrorRL) / (currentTime - prevTimeRL); // error slope
                        currentPower = getCurrentPower(currentTargetSpeedRL, Kp, Ki, Kd, acculErrorRL, currentError, errorSlope); // apply PID correction
                    } else { // at the first point, use Kp only
                        currentPower = motorPowers[2] + currentError * Kp;
                    }
                    if (currentPower > Math.abs(motorPowers[2])) currentPower = Math.abs(motorPowers[2]);
                    if (currentPower < -Math.abs(motorPowers[2])) currentPower = -Math.abs(motorPowers[2]);
                    rearLeft.setPower(currentPower);

                    prevErrorRL = currentError;
                    prevTimeRL = currentTime;
                    prevCountRL = currentCountRL;
                }
            }
            // Rear right motor control
            if (!isMotorRRDone) {
                currentCountRR = rearRight.getCurrentPosition();
                currentTime = ((double) timer.nanoseconds()) * 1.0e-9 - startTime;
                targetCountRR = getTargetTickCount(tickCount[3], motorPowers[3], rampTime, currentTime);
                currentTargetSpeedRR = getTargetSpeed(tickCount[3], motorPowers[3], rampTime, currentTime);
                if (initialized) { // check if the motor is rotating
                    isMotorRRNotMoving = Math.abs(currentCountRR - prevCountRR) < timeOutThreshold;
                }
                if (tickCount[3] == 0 || currentCountFL * (tickCount[3] / Math.abs(tickCount[3])) >= Math.abs(tickCount[3])) {
                    isMotorRRDone = true;
                    isMotorRRNotMoving = true;
                    rearRight.setPower(0.0);

                    prevErrorRR = 0;
                    prevTimeRR = 0;
                    prevCountRR = 0;
                } else {
                    currentError = targetCountRR - currentCountRR;
                    if (initialized) { // after the first point, the previous data is valid
                        acculErrorRR =
                                acculErrorRR * alpha
                                        + currentError * (currentTime - prevTimeRR); // integrate error
                        errorSlope = (currentError - prevErrorRR) / (currentTime - prevTimeRR); // error slope
                        currentPower = getCurrentPower(currentTargetSpeedRR, Kp, Ki, Kd, acculErrorRR, currentError, errorSlope); // apply PID correction
                    } else { // at the first point, use Kp only
                        currentPower = motorPowers[3] + currentError * Kp;
                    }
                    if (currentPower > Math.abs(motorPowers[3])) currentPower = Math.abs(motorPowers[3]);
                    if (currentPower < -Math.abs(motorPowers[3])) currentPower = -Math.abs(motorPowers[3]);
                    rearRight.setPower(currentPower);

                    prevErrorRR = currentError;
                    prevTimeRR = currentTime;
                    prevCountRR = currentCountRR;
                }
            }
            // Watch for timeout
            initialized = true; // enable Ki and Kd terms
            if (isMotorFLNotMoving && isMotorFRNotMoving && isMotorRLNotMoving && isMotorRRNotMoving) {
                if (isTimeOutStarted) {
                    if (currentTime - timeOutStartedTime > timeOutPeriod) {
                        isTimeOutExceeded = true;
                    }
                } else { // time out was not started yet
                    isTimeOutStarted = true;
                    timeOutStartedTime = currentTime;
                }
            } else {
                isTimeOutStarted = false;
                isTimeOutExceeded = false;
            }
            String output = "";
            try {
                output = "Motor Status: " + isMotorFLDone + " " + isMotorFRDone + " " +
                        isMotorRLDone + " " + isMotorRRDone + "\nTimeout Started: " + isTimeOutStarted
                        + "\nTimeout Exceeded: " + isTimeOutExceeded + "\nTime out period: " +
                        timeOutPeriod + "\nTime out started time: " + timeOutStartedTime +
                        "\nTime out threshold: " + timeOutThreshold + "\nacculError: " + acculErrorFL + " "
                        + acculErrorFR + " " + acculErrorRL + " " + acculErrorRR + "\nprevError: " + prevErrorFL + " "
                        + prevErrorFR + " " + prevErrorRL + " " + prevErrorRR + "\ninitialized: " + initialized
                        + "\ncurrentCount: " + currentCountFL + " " + currentCountFR + " " + currentCountRL + " " + currentCountRR
                        + "\ntargetCount: " + targetCountFL + " " + targetCountFR + " " + targetCountRL + " " + targetCountRR;
            }
            catch (Exception ignored) {
                output = "Motor Status: " + isMotorFLDone + " " + isMotorFRDone + " " +
                        isMotorRLDone + " " + isMotorRRDone + "\nTimeout Started: " + isTimeOutStarted
                        + "\nTimeout Exceeded: " + isTimeOutExceeded + "\nTime out period: " +
                        timeOutPeriod + "\nTime out started time: " + timeOutStartedTime +
                        "\nTime out threshold: " + timeOutThreshold + "\nacculError: " + acculErrorFL + " "
                        + acculErrorFR + " " + acculErrorRL + " " + acculErrorRR + "\nprevError: " + prevErrorFL + " "
                        + prevErrorFR + " " + prevErrorRL + " " + prevErrorRR + "\ninitialized: " + initialized;
            }
            try {
                Log.v(TAG, "motorEnc: " + output);
            }
            catch (RuntimeException ignored) {

            }
        }
    }

    private static double getCurrentPower(double maxPower, double Kp, double Ki, double Kd, double acculErrorFR, double currentError, double errorSlope) {
        return maxPower
                + currentError * Kp
                + acculErrorFR * Ki
                + errorSlope * Kd;
    }

    /**
     * Gets the target tick count
     *
     * @param tickCount   The tick count
     * @param speed       The speed
     * @param rampTime    The ramp time
     * @param elapsedTime The total elapsed time
     * @return The tick count
     */
    private int getTargetTickCount(int tickCount, double speed, double rampTime, double elapsedTime) {
        int targetTick;
        double speedOffset =
                speed * 0.15; // ramp up and ramp down with this speed offset so that there is no time the
        // speed is close to zero
        double speedExcess = speed - speedOffset;

        if ((double) tickCount
                < rampTime
                * (speed
                + speedOffset)) { // distance is shorter than a complete ramp up/ramp down cycle
            double halfTime =
                    (Math.sqrt(speedOffset * speedOffset + 4.0 * (double) tickCount * speedExcess / rampTime)
                            - speedOffset)
                            * rampTime
                            * 0.5
                            / speedExcess;
            if (elapsedTime < halfTime) { // during ramp up time
                targetTick =
                        (int) ((0.5 * speedExcess * elapsedTime / rampTime + speedOffset) * elapsedTime);
            } else { // during ramp downtime
                double remainTime = halfTime + halfTime - elapsedTime;
                targetTick =
                        tickCount
                                - ((int) ((0.5 * speedExcess * remainTime / rampTime + speedOffset) * remainTime));
            }
        } else { // distance is long enough to reach the cruise speed
            if (elapsedTime < rampTime) { // during ramp up time
                targetTick =
                        (int) ((0.5 * speedExcess * elapsedTime / rampTime + speedOffset) * elapsedTime);
            } else if ((double) tickCount - speedOffset * rampTime
                    > speed * elapsedTime) { // during constant speed period
                targetTick = (int) (speed * (elapsedTime - rampTime * 0.5) + 0.5 * rampTime * speedOffset);
            } else { // during ramp downtime
                double remainTime = ((double) tickCount - speedOffset * rampTime) / speed + rampTime - elapsedTime;
                targetTick =
                        tickCount
                                - ((int) ((0.5 * speedExcess * remainTime / rampTime + speedOffset) * remainTime));
            }
        }
        if (targetTick > tickCount) targetTick = tickCount;
        return targetTick;
    }

    /**
     * Gets the targets speed
     *
     * @param tickCount   the tick count
     * @param speed       the speed
     * @param rampTime    the ramp time
     * @param elapsedTime the total elapsed time
     * @return the target speed
     */
    private double getTargetSpeed(int tickCount, double speed, double rampTime, double elapsedTime) {
        double targetSpeed;
        double speedOffset =
                speed * 0.15; // ramp up and ramp down with this speed offset so that there is no time the
        // speed is close to zero
        double speedExcess = speed - speedOffset;

        if ((double) tickCount
                < rampTime
                * (speed
                + speedOffset)) { // distance is shorter than a complete ramp up/ramp down cycle
            double halfTime =
                    (Math.sqrt(speedOffset * speedOffset + 4.0 * (double) tickCount * speedExcess / rampTime)
                            - speedOffset)
                            * rampTime
                            * 0.5
                            / speedExcess;
            if (elapsedTime < halfTime) { // during ramp up time
                targetSpeed = speedExcess * elapsedTime / rampTime + speedOffset;
            } else { // during ramp downtime
                double remainTime = halfTime + halfTime - elapsedTime;
                targetSpeed = speedExcess * remainTime / rampTime + speedOffset;
            }
        } else { // distance is long enough to reach the cruise speed
            if (elapsedTime < rampTime) { // during ramp up time
                targetSpeed = speedExcess * elapsedTime / rampTime + speedOffset;
            } else if ((double) tickCount - speedOffset * rampTime
                    > speed * elapsedTime) { // during constant speed period
                targetSpeed = speed;
            } else { // during ramp downtime
                double remainTime = ((double) tickCount - speedOffset * rampTime) / speed + rampTime - elapsedTime;
                targetSpeed = speedExcess * remainTime / rampTime + speedOffset;
            }
        }
        if (targetSpeed < speedOffset) targetSpeed = speedOffset;
        return targetSpeed;
    }

    /**
     * Moves to the position specified by the vector.
     *
     * @param v          The position to move to
     * @param motorSpeed the peak motor speed to pass to the PID
     */
    public void moveVector(Vector v, double turnAngle, double motorSpeed) {
        double distance = v.distance(new Vector(0, 0));
        double angle = Math.atan2(v.getY(), v.getX()) - (Math.PI / 4.);
        int[] calcMotorDistancesTicks = new int[4];
        // Order is: FL, FR, RL, RR

        calcMotorDistancesTicks[0] = (int)((distance * Math.cos(angle)) * COUNTS_PER_MM * COUNTS_CORRECTION_Y);
        calcMotorDistancesTicks[0] -= (int)(turnAngle * COUNTS_PER_DEGREE);
        calcMotorDistancesTicks[1] = (int)((distance * Math.sin(angle)) * COUNTS_PER_MM * COUNTS_CORRECTION_X);
        calcMotorDistancesTicks[1] += (int)(turnAngle * COUNTS_PER_DEGREE);
        calcMotorDistancesTicks[2] = (int)((distance * Math.sin(angle)) * COUNTS_PER_MM * COUNTS_CORRECTION_X);
        calcMotorDistancesTicks[2] -= (int)(turnAngle * COUNTS_PER_DEGREE);
        calcMotorDistancesTicks[3] = (int)((distance * Math.cos(angle)) * COUNTS_PER_MM * COUNTS_CORRECTION_Y);
        calcMotorDistancesTicks[3] += (int)(turnAngle * COUNTS_PER_DEGREE);
        // Get largest motor dist
        double largestMotorsTicks = calcMotorDistancesTicks[0];
        for (int i = 1; i < 4; i++) {
            if (calcMotorDistancesTicks[i] > largestMotorsTicks) {
                largestMotorsTicks = calcMotorDistancesTicks[i];
            }
        }
        // Calc relative motor max powers
        double[] calcMotorPowers = {calcMotorDistancesTicks[0] / largestMotorsTicks,
                                    calcMotorDistancesTicks[1] / largestMotorsTicks,
                                    calcMotorDistancesTicks[2] / largestMotorsTicks,
                                    calcMotorDistancesTicks[3] / largestMotorsTicks};
        Log.v(TAG, "MOTOR_TICKDIST_CALC: [" + calcMotorDistancesTicks[0] + ", " + calcMotorDistancesTicks[1] + ", " + calcMotorDistancesTicks[2] + ", " + calcMotorDistancesTicks[3] + "]");
        Log.v(TAG, "MOTOR_MAXPOWERS_CALC: [" + calcMotorPowers[0] + ", " + calcMotorPowers[1] + ", " + calcMotorPowers[2] + ", " + calcMotorPowers[3] + "]");
        allMotorPIDControl(
                calcMotorDistancesTicks,
                calcMotorPowers,
                motorRampTime,
                motorKp,
                motorKi,
                motorKd);
        robotCurrentPosX += distance * Math.cos((robotCurrentAngle + angle) * Math.PI / 180.0);
        robotCurrentPosY += distance * Math.sin((robotCurrentAngle + angle) * Math.PI / 180.0);
        if (vtd != null)
            vtd.setTheoreticalPosition(calcBoundingBoxOfRobot(robotCurrentPosX, robotCurrentPosY));
//        if (wtd != null)
//            wtd.setPosition(new Coordinate(robotCurrentPosX, robotCurrentPosY));
        stop();
        logMovement();
    }

    public static BoundingBox calcBoundingBoxOfRobot(double robotCurrentPosX, double robotCurrentPosY) {
        Precision.DoubleEquivalence precision = Precision.doubleEquivalenceOfEpsilon(1e-6);
        double xOff = Robot.length/2;
        double yOff = Robot.width/2;
        LinePath path = LinePath.builder(precision)
                .append(Vector2D.of(robotCurrentPosX - xOff*mmPerInch, robotCurrentPosY - yOff*mmPerInch))
                .append(Vector2D.of(robotCurrentPosX - xOff*mmPerInch, robotCurrentPosY + yOff*mmPerInch))
                .append(Vector2D.of(robotCurrentPosX + xOff*mmPerInch, robotCurrentPosY + yOff*mmPerInch))
                .append(Vector2D.of(robotCurrentPosX + xOff*mmPerInch, robotCurrentPosY - yOff*mmPerInch))
                .build(true);
        return new BoundingBox(ConvexArea.convexPolygonFromPath(path));
    }

    public void moveVector(Vector v) {
        moveVector(v, 0, ANGULAR_V_MAX_NEVERREST_20);
    }
    public void moveVector(Vector v, double angle) {
        moveVector(v, angle, ANGULAR_V_MAX_NEVERREST_20);
    }


    public double getRobotCurrentPosX() {
        return robotCurrentPosX;
    }

    public double getRobotCurrentPosY() {
        return robotCurrentPosY;
    }

    public void moveVectorOdometry(Vector v, double turnAngle, double motorSpeed) {
        resetOdometry();
        moveVector(v, turnAngle, motorSpeed);
        correctAngleOdometry();
        if (odometryCountB * ODOMETRY_mm_PER_COUNT > 25.0) {
            moveLeft(odometryCountB * ODOMETRY_mm_PER_COUNT);
        }
        if (odometryCountB * ODOMETRY_mm_PER_COUNT < -25.0) {
            moveRight(-odometryCountB * ODOMETRY_mm_PER_COUNT);
        }
        double offsetY = (((double) odometryCountR) + ((double) odometryCountL)) * 0.5;
        if (offsetY * ODOMETRY_mm_PER_COUNT > 25.0) {
            moveBackward(offsetY * ODOMETRY_mm_PER_COUNT);
        }
        if (offsetY * ODOMETRY_mm_PER_COUNT < -25.0) {
            moveForward(-offsetY * ODOMETRY_mm_PER_COUNT);
        }
    }

    public void moveVectorOdometry(Vector v) {
        moveVectorOdometry(v, 0, ANGULAR_V_MAX_NEVERREST_20);
    }
    public void moveVectorOdometry(Vector v, double angle) {
        moveVectorOdometry(v, angle, ANGULAR_V_MAX_NEVERREST_20);
    }

}