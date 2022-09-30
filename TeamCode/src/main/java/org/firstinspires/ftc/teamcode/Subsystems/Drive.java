package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

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
    private static final double MOTOR_TICK_PER_REV_YELLOJACKET312 = 537.6;
    private static final double GOBUILDA_MECANUM_DIAMETER_MM = 96.0;
    private static final double COUNTS_PER_MM =
            (MOTOR_TICK_PER_REV_YELLOJACKET312 * DRIVE_GEAR_REDUCTION)
                    / (GOBUILDA_MECANUM_DIAMETER_MM * Math.PI);
    private static final double WHEEL_DIAMETER_INCHES = 100.0 / 25.4; // For figuring circumference
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
     * Default drive speed
     */
    private static final double DRIVE_SPEED = 0.40;
    private static final double DRIVE_SPEED_X = 0.35;
    private static final double DRIVE_SPEED_Y = 0.40;
    /**
     * Default turn speed
     */
    private static final double TURN_SPEED = 0.40;
    private static final double ROBOT_INIT_POS_X = 15.0;
    private static final double ROBOT_INIT_POS_Y = 15.0;
    private static final double ROBOT_INIT_ANGLE = 45.0;
    /**
     * Number of millimeters per an Inch
     */
    private static final double mmPerInch = 25.4;
    private static boolean driveFullPower = false;
    private static final double motorKp = 0.015;
    private static final double motorKi = 0.02;
    private static final double motorKd = 0.0003;
    private static final double motorRampTime = 0.3;
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

    /**
     * Inited the drive subsystem
     *
     * @param frontLeft   The front left motor
     * @param frontRight  The front right motor
     * @param rearLeft    The rear left motor
     * @param rearRight   The rear right motor
     * @param imu         The imu
     * @param telemetry   The telemetry
     * @param hardwareMap The hardware map for getting the motors
     * @param timer       The timer for the elapsed time
     */
    public Drive(DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx rearLeft, DcMotorEx rearRight,
                 Telemetry telemetry,
                 HardwareMap hardwareMap,
                 ElapsedTime timer) {
        super(telemetry, hardwareMap, timer);
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.rearLeft = rearLeft;
        this.rearRight = rearRight;
//        this.odL = odL;
//        this.odB = odB;
//        this.odR = odR;
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Gets the left odometry
     * @return The left odometry
     */
    private int getOdometryCountL() {
        odometryCountL = -odL.getCurrentPosition() - odometryCountOffsetL;
        return odometryCountL;
    }

    /**
     * Gets the back odometry
     * @return The back odometry
     */
    private int getOdometryCountB() {
        odometryCountB = odB.getCurrentPosition() - odometryCountOffsetB;
        return odometryCountB;
    }

    /**
     * Gets the right odometry
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
     * Stops all drive motors
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
     * Turns with the specified power
     *
     * @param power The power to turn by.
     */
    public void turn(double power) {
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        rearLeft.setPower(-power);
        rearRight.setPower(power);
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
        double r = Math.hypot(leftStickX, leftStickY);
        double robotAngle = Math.atan2(leftStickY, leftStickX) - Math.PI / 4;
        double lrPower = r * Math.sin(robotAngle) + rightStickX;
        double lfPower = r * Math.cos(robotAngle) + rightStickX;
        double rrPower = r * Math.cos(robotAngle) - rightStickX;
        double rfPower = r * Math.sin(robotAngle) - rightStickX;
        return new double[]{lfPower, rfPower, lrPower, rrPower};
    }

    /**
     * Sets the drive power
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
    public void setDrivePower(double[] powers) {
        frontLeft.setPower(powers[0]);
        frontRight.setPower(powers[1]);
        rearLeft.setPower(powers[2]);
        rearRight.setPower(powers[3]);
    }

    /**
     * Set the full power
     *
     * @param fullPower full power boolean
     */
    public void setDriveFullPower(boolean fullPower) {
        driveFullPower = fullPower;
    }

    /**
     * Set the target position
     *
     * @param targetPosition the target position
     */
    public void setTargetPosition(int targetPosition) {
        frontLeft.setTargetPosition(targetPosition);
        frontRight.setTargetPosition(targetPosition);
        rearLeft.setTargetPosition(targetPosition);
        rearRight.setTargetPosition(targetPosition);
    }

    /**
     * Get the current positions of all the motors
     *
     * @return the list of all the positions of the drive motors
     */
    public int[] getCurrentPositions() {
        return new int[]{
                frontLeft.getCurrentPosition() - encoderOffsetFL,
                frontRight.getCurrentPosition() - encoderOffsetFR,
                rearLeft.getCurrentPosition() - encoderOffsetRL,
                rearRight.getCurrentPosition() - encoderOffsetRR
        };
    }

    public int[] getDriveMotorEncoders() {
        return new int[]{
                frontLeft.getCurrentPosition(),
                frontRight.getCurrentPosition(),
                rearLeft.getCurrentPosition(),
                rearRight.getCurrentPosition()
        };
    }

    /**
     * Resets the motor encoders for the drivetrain motors.
     */
    public void resetDriveMotorEncoders() {
        encoderOffsetFL = frontLeft.getCurrentPosition();
        encoderOffsetFR = frontRight.getCurrentPosition();
        encoderOffsetRL = rearLeft.getCurrentPosition();
        encoderOffsetRR = rearRight.getCurrentPosition();
    }

    /**
     * Positive encoder values correspond to rightward robot movement
     */
    public void strafe(int targetPosition) {
        frontLeft.setTargetPosition(targetPosition);
        frontRight.setTargetPosition(-targetPosition);
        rearLeft.setTargetPosition(-targetPosition);
        rearRight.setTargetPosition(targetPosition);
    }

    /**
     * Turns the robot by the specified angle, ticks and angles are equivalent. Use this method to turn the robot instead
     * of {@link #turnByTick(double, double)}
     *
     * @param angle The angle to turn by. Positive corresponds to counterclockwise
     */
    public void turnByAngle(double angle) {
        if (angle > 0.0) {
            allMotorPIDControl(
                    (int) (angle * COUNTS_PER_DEGREE),
                    TURN_SPEED * ANGULAR_V_MAX_NEVERREST_20,
                    ANGULAR_V_MAX_NEVERREST_20,
                    motorRampTime,
                    false,
                    true,
                    false,
                    true,
                    motorKp,
                    motorKi,
                    motorKd);
        } else {
            allMotorPIDControl(
                    (int) (-angle * COUNTS_PER_DEGREE),
                    TURN_SPEED * ANGULAR_V_MAX_NEVERREST_20,
                    ANGULAR_V_MAX_NEVERREST_20,
                    motorRampTime,
                    true,
                    false,
                    true,
                    false,
                    motorKp,
                    motorKi,
                    motorKd);
        }
        //        robotCurrentPosX += ROBOT_HALF_LENGTH *
        // (Math.cos((robotCurrentAngle+degrees)*Math.PI/180.0)
        //                - Math.cos(robotCurrentAngle*Math.PI/180.0));
        //        robotCurrentPosY += ROBOT_HALF_LENGTH *
        // (Math.sin((robotCurrentAngle+degrees)*Math.PI/180.0)
        //                - Math.sin(robotCurrentAngle*Math.PI/180.0));
        robotCurrentAngle += angle;
        // Display it for the driver.
//        telemetry.addData("turnRobot", "turn to %7.2f degrees", robotCurrentAngle);
//        telemetry.update();
    }

    /**
     * Turns the robot by tick. Use {@link #turnByAngle(double)}
     *
     * @param power the motor power
     * @param angle the angle in ticks
     */
    public void turnByTick(double power, double angle) {
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setTargetPosition(0);
        frontRight.setTargetPosition(0);
        rearLeft.setTargetPosition(0);
        rearRight.setTargetPosition(0);

        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (driveFullPower) {
            setDrivePower(1.0);
        } else {
            setDrivePower(power);
        }
        // convert from degrees to motor counts
        int tickCount = (int) (angle * COUNTS_PER_DEGREE);
        frontLeft.setTargetPosition(-tickCount);
        frontRight.setTargetPosition(tickCount);
        rearLeft.setTargetPosition(-tickCount);
        rearRight.setTargetPosition(tickCount);
        startTime = timer.nanoseconds();
        while (frontLeft.isBusy() && frontRight.isBusy() && rearLeft.isBusy() && rearRight.isBusy()) {
            logDriveEncoders();
            checkAndStopMotors();
        }
        stop();
        logDriveEncoders();
    }

    /**
     * Turns the robot by the specified angle.
     *
     * @param degrees The angle to turn by.
     */
    public void turnRobot(double degrees) {
        this.turnByAngle(degrees);
        //        robotCurrentPosX += ROBOT_HALF_LENGTH *
        // (Math.cos((robotCurrentAngle+degrees)*Math.PI/180.0)
        //                - Math.cos(robotCurrentAngle*Math.PI/180.0));
        //        robotCurrentPosY += ROBOT_HALF_LENGTH *
        // (Math.sin((robotCurrentAngle+degrees)*Math.PI/180.0)
        //                - Math.sin(robotCurrentAngle*Math.PI/180.0));
        robotCurrentAngle += degrees;
        // Display it for the driver.
        telemetry.addData("turnRobot", "turn to %7.2f degrees", robotCurrentAngle);
        telemetry.update();
    }

    /**
     * 2D move to position
     *
     * @param power           motor power
     * @param targetPositionX target x coordinate
     * @param targetPositionY target y coordinate
     */
    public void moveToPos2D(double power, double targetPositionX, double targetPositionY) {
        // move to X, Y position relative to the robot coordinate system
        // the center of robot is 0,0
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setTargetPosition(0);
        frontRight.setTargetPosition(0);
        rearLeft.setTargetPosition(0);
        rearRight.setTargetPosition(0);

        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        // convert from inches to motor counts
        // correct for X and Y motion asymmetry
        double distanceCountX, distanceCountY;
        distanceCountX = targetPositionX * COUNTS_PER_MM * COUNTS_CORRECTION_X;
        distanceCountY = targetPositionY * COUNTS_PER_MM * COUNTS_CORRECTION_Y;
        if (driveFullPower) {
            setPower2D(distanceCountX, distanceCountY, 1.0);
        } else {
            setPower2D(distanceCountX, distanceCountY, power);
        }
        setTargetPosition2D(distanceCountX, distanceCountY);
        startTime = timer.nanoseconds();
        while (frontLeft.isBusy() && frontRight.isBusy() && rearLeft.isBusy() && rearRight.isBusy()) {
            logDriveEncoders();
            checkAndStopMotors();
        }
        stop();
        logDriveEncoders();
    }

    /**
     * distribute power appropriately according to the direction of motion.
     *
     * @param targetPositionX The x target position
     * @param targetPositionY The y target position
     * @param motorPower      the motor power
     */
    public void setPower2D(double targetPositionX, double targetPositionY, double motorPower) {
        double[] motorPowers = calcMotorPowers2D(targetPositionX, targetPositionY, motorPower);
        rearLeft.setPower(motorPowers[0]);
        frontLeft.setPower(motorPowers[1]);
        rearRight.setPower(motorPowers[2]);
        frontRight.setPower(motorPowers[3]);
    }

    /**
     * set motor rotation targets appropriately according to the direction of motion.
     *
     * @param targetPositionX The x target position
     * @param targetPositionY The y target position
     */
    public void setTargetPosition2D(double targetPositionX, double targetPositionY) {
        //        frontLeft.setTargetPosition((int)  ((+ targetPositionX +
        // targetPositionY)*Math.sqrt(2.0)));
        //        frontRight.setTargetPosition((int) ((- targetPositionX +
        // targetPositionY)*Math.sqrt(2.0)));
        //        rearLeft.setTargetPosition((int)   ((- targetPositionX +
        // targetPositionY)*Math.sqrt(2.0)));
        //        rearRight.setTargetPosition((int)  ((+ targetPositionX +
        // targetPositionY)*Math.sqrt(2.0)));
        frontLeft.setTargetPosition((int) (targetPositionX + targetPositionY));
        frontRight.setTargetPosition((int) (-targetPositionX + targetPositionY));
        rearLeft.setTargetPosition((int) (-targetPositionX + targetPositionY));
        rearRight.setTargetPosition((int) (targetPositionX + targetPositionY));
    }

    /**
     * targetPositionX and targetPositionY determine the direction of movement
     * motorPower determines the magnitude of motor power
     *
     * @param targetPositionX The target x position
     * @param targetPositionY the target y position
     * @param motorPower      the motor power
     * @return a list with the motor powers
     */
    public double[] calcMotorPowers2D(double targetPositionX, double targetPositionY, double motorPower) {
        double angleScale = Math.abs(targetPositionX) + Math.abs(targetPositionY);
        double lrPower = motorPower * (-targetPositionX + targetPositionY) / angleScale;
        double lfPower = motorPower * (targetPositionX + targetPositionY) / angleScale;
        return new double[]{lrPower, lfPower, lfPower, lrPower}; // rrPower=lfPower and rfPower=lrPower
    }

    public void moveToPosABS(double targetPositionX, double targetPositionY) {
        // move to (targetPositionX, targetPositionY) in absolute field coordinate
        double deltaX = targetPositionX - robotCurrentPosX; // in absolute field coordinate
        double deltaY = targetPositionY - robotCurrentPosY; // in absolute field coordinate
        double distanceCountX, distanceCountY; // distance in motor count in robot coordinate
        // rotate vector from field coordinate to robot coordinate
        distanceCountX =
                deltaX * Math.cos((robotCurrentAngle - 90.0) * Math.PI / 180.0)
                        + deltaY * Math.sin((robotCurrentAngle - 90.0) * Math.PI / 180.0);
        distanceCountY =
                deltaX * Math.cos(robotCurrentAngle * Math.PI / 180.0)
                        + deltaY * Math.sin(robotCurrentAngle * Math.PI / 180.0);
        this.moveToPos2D(DRIVE_SPEED, distanceCountX, distanceCountY);
        robotCurrentPosX = targetPositionX;
        robotCurrentPosY = targetPositionY;
        // Display it for the driver.
        telemetry.addData(
                "moveToPosABS", "move to %7.2f, %7.2f", robotCurrentPosX, robotCurrentPosY);
        telemetry.update();
        //        sleep(100);
    }

    public void moveToPosREL(double targetPositionX, double targetPositionY) {
        // move to (targetPositionX, targetPositionY) in relative robot coordinate
        this.moveToPos2D(DRIVE_SPEED, targetPositionX, targetPositionY);
        robotCurrentPosX +=
                targetPositionY * Math.cos(robotCurrentAngle * Math.PI / 180.0)
                        + targetPositionX * Math.cos((robotCurrentAngle - 90.0) * Math.PI / 180.0);
        robotCurrentPosY +=
                targetPositionY * Math.sin(robotCurrentAngle * Math.PI / 180.0)
                        + targetPositionX * Math.sin((robotCurrentAngle - 90.0) * Math.PI / 180.0);
        // Display it for the driver.
        telemetry.addData(
                "moveToPosREL", "move to %7.2f, %7.2f", robotCurrentPosX, robotCurrentPosY);
        telemetry.update();
        //        sleep(100);
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
        //        this.moveToPos2D(motorSpeed, 0.0, distance);
        allMotorPIDControl(
                (int) (distance * COUNTS_PER_MM * COUNTS_CORRECTION_Y),
                motorSpeed * ANGULAR_V_MAX_NEVERREST_20,
                ANGULAR_V_MAX_NEVERREST_20,
                motorRampTime,
                true,
                true,
                true,
                true,
                motorKp,
                motorKi,
                motorKd);
        robotCurrentPosX += distance * Math.cos(robotCurrentAngle * Math.PI / 180.0);
        robotCurrentPosY += distance * Math.sin(robotCurrentAngle * Math.PI / 180.0);
        // Display it for the driver.
        telemetry.addData(
                "moveForward", "move to %7.2f, %7.2f", robotCurrentPosX, robotCurrentPosY);
        updateOdometry();
        telemetry.addData(
                "odometry",
                " L %7.2f R %7.2f B %7.2f",
                odometryCountL * ODOMETRY_mm_PER_COUNT,
                odometryCountR * ODOMETRY_mm_PER_COUNT,
                odometryCountB * ODOMETRY_mm_PER_COUNT);
        telemetry.update();
        //        sleep(1000);
        double angleError =
                (((double) odometryCountR) - ((double) odometryCountL))
                        * 0.5
                        * ODOMETRY_mm_PER_COUNT
                        * (180.0 / 3.14159265)
                        / ODOMETRY_RADIUS_X;
        turnByAngle(-angleError);
        updateOdometry();
        telemetry.addData("correction angle", " %7.2f", -angleError);
        telemetry.addData(
                "odometry",
                " L %7.2f R %7.2f B %7.2f",
                odometryCountL * ODOMETRY_mm_PER_COUNT,
                odometryCountR * ODOMETRY_mm_PER_COUNT,
                odometryCountB * ODOMETRY_mm_PER_COUNT);
        telemetry.update();
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
        //        this.moveToPos2D(motorSpeed, 0.0, distance);
        allMotorPIDControl(
                (int) (distance * COUNTS_PER_MM * COUNTS_CORRECTION_Y),
                motorSpeed * ANGULAR_V_MAX_NEVERREST_20,
                ANGULAR_V_MAX_NEVERREST_20,
                motorRampTime,
                true,
                true,
                true,
                true,
                motorKp,
                motorKi,
                motorKd);
        robotCurrentPosX += distance * Math.cos(robotCurrentAngle * Math.PI / 180.0);
        robotCurrentPosY += distance * Math.sin(robotCurrentAngle * Math.PI / 180.0);
        // Display it for the driver.
//        telemetry.addData(
//                "moveForward", "move to %7.2f, %7.2f", robotCurrentPosX, robotCurrentPosY);
//        telemetry.update();
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
        //        this.moveToPos2D(motorSpeed, 0.0, -distance);
        allMotorPIDControl(
                (int) (distance * COUNTS_PER_MM * COUNTS_CORRECTION_Y),
                motorSpeed * ANGULAR_V_MAX_NEVERREST_20,
                ANGULAR_V_MAX_NEVERREST_20,
                motorRampTime,
                false,
                false,
                false,
                false,
                motorKp,
                motorKi,
                motorKd);
        robotCurrentPosX += distance * Math.cos((robotCurrentAngle + 180.0) * Math.PI / 180.0);
        robotCurrentPosY += distance * Math.sin((robotCurrentAngle + 180.0) * Math.PI / 180.0);
        // Display it for the driver.
        telemetry.addData(
                "moveBackward", "move to %7.2f, %7.2f", robotCurrentPosX, robotCurrentPosY);
        updateOdometry();
        telemetry.addData(
                "odometry",
                " L %7.2f R %7.2f B %7.2f",
                odometryCountL * ODOMETRY_mm_PER_COUNT,
                odometryCountR * ODOMETRY_mm_PER_COUNT,
                odometryCountB * ODOMETRY_mm_PER_COUNT);
        telemetry.update();
        //        sleep(1000);
        double angleError =
                (((double) odometryCountR) - ((double) odometryCountL))
                        * 0.5
                        * ODOMETRY_mm_PER_COUNT
                        * (180.0 / 3.14159265)
                        / ODOMETRY_RADIUS_X;
        turnByAngle(-angleError);
        updateOdometry();
        telemetry.addData("correction angle", " %7.2f", -angleError);
        telemetry.addData(
                "odometry",
                " L %7.2f R %7.2f B %7.2f",
                odometryCountL * ODOMETRY_mm_PER_COUNT,
                odometryCountR * ODOMETRY_mm_PER_COUNT,
                odometryCountB * ODOMETRY_mm_PER_COUNT);
        telemetry.update();
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
        //        this.moveToPos2D(motorSpeed, 0.0, -distance);
        allMotorPIDControl(
                (int) (distance * COUNTS_PER_MM * COUNTS_CORRECTION_Y),
                motorSpeed * ANGULAR_V_MAX_NEVERREST_20,
                ANGULAR_V_MAX_NEVERREST_20,
                motorRampTime,
                false,
                false,
                false,
                false,
                motorKp,
                motorKi,
                motorKd);
        robotCurrentPosX += distance * Math.cos((robotCurrentAngle + 180.0) * Math.PI / 180.0);
        robotCurrentPosY += distance * Math.sin((robotCurrentAngle + 180.0) * Math.PI / 180.0);
        // Display it for the driver.
//        telemetry.addData(
//                "moveBackward", "move to %7.2f, %7.2f", robotCurrentPosX, robotCurrentPosY);
//        telemetry.update();
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
        //        this.moveToPos2D(motorSpeed, -distance, 0.0);
        allMotorPIDControl(
                (int) (distance * COUNTS_PER_MM * COUNTS_CORRECTION_X),
                motorSpeed * ANGULAR_V_MAX_NEVERREST_20,
                ANGULAR_V_MAX_NEVERREST_20,
                motorRampTime,
                false,
                true,
                true,
                false,
                motorKp,
                motorKi,
                motorKd);
        robotCurrentPosX += distance * Math.cos((robotCurrentAngle + 90.0) * Math.PI / 180.0);
        robotCurrentPosY += distance * Math.sin((robotCurrentAngle + 90.0) * Math.PI / 180.0);
        // Display it for the driver.
        telemetry.addData(
                "moveLeft", "move to %7.2f, %7.2f", robotCurrentPosX, robotCurrentPosY);
        updateOdometry();
        telemetry.addData(
                "odometry",
                " L %7.2f R %7.2f B %7.2f",
                odometryCountL * ODOMETRY_mm_PER_COUNT,
                odometryCountR * ODOMETRY_mm_PER_COUNT,
                odometryCountB * ODOMETRY_mm_PER_COUNT);
        telemetry.update();
        //        sleep(1000);
        double angleError =
                (((double) odometryCountR) - ((double) odometryCountL))
                        * 0.5
                        * ODOMETRY_mm_PER_COUNT
                        * (180.0 / 3.14159265)
                        / ODOMETRY_RADIUS_X;
        turnByAngle(-angleError);
        updateOdometry();
        telemetry.addData("correction angle", " %7.2f", -angleError);
        telemetry.addData(
                "odometry",
                " L %7.2f R %7.2f B %7.2f",
                odometryCountL * ODOMETRY_mm_PER_COUNT,
                odometryCountR * ODOMETRY_mm_PER_COUNT,
                odometryCountB * ODOMETRY_mm_PER_COUNT);
        telemetry.update();
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
        //        this.moveToPos2D(motorSpeed, -distance, 0.0);
        allMotorPIDControl(
                (int) (distance * COUNTS_PER_MM * COUNTS_CORRECTION_X),
                motorSpeed * ANGULAR_V_MAX_NEVERREST_20,
                ANGULAR_V_MAX_NEVERREST_20,
                motorRampTime,
                false,
                true,
                true,
                false,
                motorKp,
                motorKi,
                motorKd);
        robotCurrentPosX += distance * Math.cos((robotCurrentAngle + 90.0) * Math.PI / 180.0);
        robotCurrentPosY += distance * Math.sin((robotCurrentAngle + 90.0) * Math.PI / 180.0);
        // Display it for the driver.
        telemetry.addData(
                "moveLeft", "move to %7.2f, %7.2f", robotCurrentPosX, robotCurrentPosY);
        telemetry.update();
        //        sleep(100);
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
        //        this.moveToPos2D(motorSpeed, distance, 0.0);
        allMotorPIDControl(
                (int) (distance * COUNTS_PER_MM * COUNTS_CORRECTION_X),
                motorSpeed * ANGULAR_V_MAX_NEVERREST_20,
                ANGULAR_V_MAX_NEVERREST_20,
                motorRampTime,
                true,
                false,
                false,
                true,
                motorKp,
                motorKi,
                motorKd);
        robotCurrentPosX += distance * Math.cos((robotCurrentAngle - 90.0) * Math.PI / 180.0);
        robotCurrentPosY += distance * Math.sin((robotCurrentAngle - 90.0) * Math.PI / 180.0);
        // Display it for the driver.
        telemetry.addData(
                "moveRight", "move to %7.2f, %7.2f", robotCurrentPosX, robotCurrentPosY);
        updateOdometry();
        telemetry.addData(
                "odometry",
                " L %7.2f R %7.2f B %7.2f",
                odometryCountL * ODOMETRY_mm_PER_COUNT,
                odometryCountR * ODOMETRY_mm_PER_COUNT,
                odometryCountB * ODOMETRY_mm_PER_COUNT);
        telemetry.update();
        //        sleep(1000);
        double angleError =
                (((double) odometryCountR) - ((double) odometryCountL))
                        * 0.5
                        * ODOMETRY_mm_PER_COUNT
                        * (180.0 / 3.14159265)
                        / ODOMETRY_RADIUS_X;
        turnByAngle(-angleError);
        updateOdometry();
        telemetry.addData("correction angle", " %7.2f", -angleError);
        telemetry.addData(
                "odometry",
                " L %7.2f R %7.2f B %7.2f",
                odometryCountL * ODOMETRY_mm_PER_COUNT,
                odometryCountR * ODOMETRY_mm_PER_COUNT,
                odometryCountB * ODOMETRY_mm_PER_COUNT);
        telemetry.update();
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
        //        this.moveToPos2D(motorSpeed, distance, 0.0);
        allMotorPIDControl(
                (int) (distance * COUNTS_PER_MM * COUNTS_CORRECTION_X),
                motorSpeed * ANGULAR_V_MAX_NEVERREST_20,
                ANGULAR_V_MAX_NEVERREST_20,
                motorRampTime,
                true,
                false,
                false,
                true,
                motorKp,
                motorKi,
                motorKd);
        robotCurrentPosX += distance * Math.cos((robotCurrentAngle - 90.0) * Math.PI / 180.0);
        robotCurrentPosY += distance * Math.sin((robotCurrentAngle - 90.0) * Math.PI / 180.0);
        // Display it for the driver.
        telemetry.addData(
                "moveRight", "move to %7.2f, %7.2f", robotCurrentPosX, robotCurrentPosY);
        telemetry.update();
        //        sleep(100);
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
     * @param motor The motor to calculate the PID Coefficients
     * @param mode The Run Mode
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
     * @param motor The motor
     * @param mode The run mode
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
     * Logs the drive encoder results to telemetry and the phone.
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
        telemetry.addData("motorEnc", output);
        telemetry.update();
    }

    /**
     * PID motor control program to ensure all four motors are synchronized
     *
     * @param tickCount:      absolute value of target tickcount of motor
     * @param peakSpeed:      peak speed of motor rotation in tick per second
     * @param maxSpeed:       max speed of motor rotation in tick per second
     * @param rampTime:       motor speed ramp up/down time in sec
     * @param motorFLForward: front left motor is forward
     * @param motorFRForward: front right motor is forward
     * @param motorRLForward: rear left motor is forward
     * @param motorRRForward: rear right motor is forward
     * @param Kp:             coefficient Kp
     * @param Ki:             coefficient Ki
     * @param Kd:             coefficient Kd by Andrew Chiang on 1/28/2020
     */
    public void allMotorPIDControl(
            int tickCount,
            double peakSpeed,
            double maxSpeed,
            double rampTime,
            boolean motorFLForward,
            boolean motorFRForward,
            boolean motorRLForward,
            boolean motorRRForward,
            double Kp,
            double Ki,
            double Kd) {
        stop();
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        int currentCount, targetCount;
        int prevCountFL = 0;
        int prevCountFR = 0;
        int prevCountRL = 0;
        int prevCountRR = 0;
        double currentError = 0.0;
        double currentTargetSpeed;
        double currentPower = 0.0;
        double alpha = 0.95;
        double startTime = ((double) timer.nanoseconds()) * 1.0e-9;
        double currentTime = 0.0;
        double errorSlope = 0.0;
        while (((!isMotorFLDone) || (!isMotorFRDone) || (!isMotorRLDone) || (!isMotorRRDone))
                && (!isTimeOutExceeded)) {
            if (!isMotorFLDone) {
                currentCount = frontLeft.getCurrentPosition(); // get current motor tick
                currentTime = ((double) timer.nanoseconds()) * 1.0e-9 - startTime; // get current time
                targetCount =
                        getTargetTickCount(
                                tickCount,
                                peakSpeed,
                                rampTime,
                                currentTime); // get integrated target tick on the speed profile
                currentTargetSpeed =
                        getTargetSpeed(
                                tickCount,
                                peakSpeed,
                                rampTime,
                                currentTime); // get the target speed on the speed profile
                if (initialized) { // check if the motor is rotating
                    isMotorFLNotMoving = Math.abs(currentCount - prevCountFL) < timeOutThreshold;
                }
                if (motorFLForward) { // tick count increasing
                    if (currentCount >= tickCount) {
                        isMotorFLDone = true;
                        isMotorFLNotMoving = true;
                        frontLeft.setPower(0.0);
                    } else {
                        currentError = currentCount - targetCount;
                        if (initialized) { // after the first point, the previous data is valid
                            acculErrorFL =
                                    acculErrorFL * alpha
                                            + currentError * (currentTime - prevTimeFL); // integrate error
                            errorSlope = (currentError - prevErrorFL) / (currentTime - prevTimeFL); // error slope
                            currentPower =
                                    currentTargetSpeed / maxSpeed
                                            - currentError * Kp
                                            - acculErrorFL * Ki
                                            - errorSlope * Kd; // apply PID correction
                        } else { // at the first point, use Kp only
                            currentPower = currentTargetSpeed / maxSpeed - currentError * Kp;
                        }
                        if (currentPower > 1.0) currentPower = 1.0;
                        if (currentPower < 0.0) currentPower = 0.0;
                        frontLeft.setPower(currentPower);
                    }
                } else { // motorFLForward is false, tick count negative and decreasing
                    if (currentCount <= -tickCount) {
                        isMotorFLDone = true;
                        isMotorFLNotMoving = true;
                        frontLeft.setPower(0.0);
                    } else {
                        currentError = -currentCount - targetCount;
                        if (initialized) { // after the first point, the previous data is valid
                            acculErrorFL =
                                    acculErrorFL * alpha
                                            + currentError * (currentTime - prevTimeFL); // integrate error
                            errorSlope = (currentError - prevErrorFL) / (currentTime - prevTimeFL); // error slope
                            currentPower =
                                    -currentTargetSpeed / maxSpeed
                                            + currentError * Kp
                                            + acculErrorFL * Ki
                                            + errorSlope * Kd; // apply PID correction
                        } else { // at the first point, use Kp only
                            currentPower = -currentTargetSpeed / maxSpeed + currentError * Kp;
                        }
                        if (currentPower < -1.0) currentPower = -1.0;
                        if (currentPower > 0.0) currentPower = 0.0;
                        frontLeft.setPower(currentPower);
                    }
                }
                prevErrorFL = currentError;
                prevTimeFL = currentTime;
                prevCountFL = currentCount;
            } // if (!isMotorFLDone)
            if (!isMotorFRDone) {
                currentCount = frontRight.getCurrentPosition();
                currentTime = ((double) timer.nanoseconds()) * 1.0e-9 - startTime;
                targetCount = getTargetTickCount(tickCount, peakSpeed, rampTime, currentTime);
                currentTargetSpeed = getTargetSpeed(tickCount, peakSpeed, rampTime, currentTime);
                if (initialized) { // check if the motor is rotating
                    isMotorFRNotMoving = Math.abs(currentCount - prevCountFR) < timeOutThreshold;
                }
                if (motorFRForward) {
                    if (currentCount >= tickCount) {
                        isMotorFRDone = true;
                        isMotorFRNotMoving = true;
                        frontRight.setPower(0.0);
                    } else {
                        currentError = currentCount - targetCount;
                        if (initialized) { // after the first point, the previous data is valid
                            acculErrorFR =
                                    acculErrorFR * alpha
                                            + currentError * (currentTime - prevTimeFR); // integrate error
                            errorSlope = (currentError - prevErrorFR) / (currentTime - prevTimeFR); // error slope
                            currentPower =
                                    currentTargetSpeed / maxSpeed
                                            - currentError * Kp
                                            - acculErrorFR * Ki
                                            - errorSlope * Kd; // apply PID correction
                        } else { // at the first point, use Kp only
                            currentPower = currentTargetSpeed / maxSpeed - currentError * Kp;
                        }
                        if (currentPower > 1.0) currentPower = 1.0;
                        if (currentPower < 0.0) currentPower = 0.0;
                        frontRight.setPower(currentPower);
                    }
                } else { // motorFRForward is false
                    if (currentCount <= -tickCount) {
                        isMotorFRDone = true;
                        isMotorFRNotMoving = true;
                        frontRight.setPower(0.0);
                    } else {
                        currentError = -currentCount - targetCount;
                        if (initialized) { // after the first point, the previous data is valid
                            acculErrorFR =
                                    acculErrorFR * alpha
                                            + currentError * (currentTime - prevTimeFR); // integrate error
                            errorSlope = (currentError - prevErrorFR) / (currentTime - prevTimeFR); // error slope
                            currentPower =
                                    -currentTargetSpeed / maxSpeed
                                            + currentError * Kp
                                            + acculErrorFR * Ki
                                            + errorSlope * Kd; // apply PID correction
                        } else { // at the first point, use Kp only
                            currentPower = -currentTargetSpeed / maxSpeed + currentError * Kp;
                        }
                        if (currentPower < -1.0) currentPower = -1.0;
                        if (currentPower > 0.0) currentPower = 0.0;
                        frontRight.setPower(currentPower);
                    }
                }
                prevErrorFR = currentError;
                prevTimeFR = currentTime;
                prevCountFR = currentCount;
            } // if (!isMotorFRDone)
            if (!isMotorRLDone) {
                currentCount = rearLeft.getCurrentPosition();
                currentTime = ((double) timer.nanoseconds()) * 1.0e-9 - startTime;
                targetCount = getTargetTickCount(tickCount, peakSpeed, rampTime, currentTime);
                currentTargetSpeed = getTargetSpeed(tickCount, peakSpeed, rampTime, currentTime);
                if (initialized) { // check if the motor is rotating
                    isMotorRLNotMoving = Math.abs(currentCount - prevCountRL) < timeOutThreshold;
                }
                if (motorRLForward) {
                    if (currentCount >= tickCount) {
                        isMotorRLDone = true;
                        isMotorRLNotMoving = true;
                        rearLeft.setPower(0.0);
                    } else {
                        currentError = currentCount - targetCount;
                        if (initialized) { // after the first point, the previous data is valid
                            acculErrorRL =
                                    acculErrorRL * alpha
                                            + currentError * (currentTime - prevTimeRL); // integrate error
                            errorSlope = (currentError - prevErrorRL) / (currentTime - prevTimeRL); // error slope
                            currentPower =
                                    currentTargetSpeed / maxSpeed
                                            - currentError * Kp
                                            - acculErrorRL * Ki
                                            - errorSlope * Kd; // apply PID correction
                        } else { // at the first point, use Kp only
                            currentPower = currentTargetSpeed / maxSpeed - currentError * Kp;
                        }
                        if (currentPower > 1.0) currentPower = 1.0;
                        if (currentPower < 0.0) currentPower = 0.0;
                        rearLeft.setPower(currentPower);
                    }
                } else { // motorFLForward is false
                    if (currentCount <= -tickCount) {
                        isMotorRLDone = true;
                        isMotorRLNotMoving = true;
                        rearLeft.setPower(0.0);
                    } else {
                        currentError = -currentCount - targetCount;
                        if (initialized) { // after the first point, the previous data is valid
                            acculErrorRL =
                                    acculErrorRL * alpha
                                            + currentError * (currentTime - prevTimeRL); // integrate error
                            errorSlope = (currentError - prevErrorRL) / (currentTime - prevTimeRL); // error slope
                            currentPower =
                                    -currentTargetSpeed / maxSpeed
                                            + currentError * Kp
                                            + acculErrorRL * Ki
                                            + errorSlope * Kd; // apply PID correction
                        } else { // at the first point, use Kp only
                            currentPower = -currentTargetSpeed / maxSpeed + currentError * Kp;
                        }
                        if (currentPower < -1.0) currentPower = -1.0;
                        if (currentPower > 0.0) currentPower = 0.0;
                        rearLeft.setPower(currentPower);
                    }
                }
                prevErrorRL = currentError;
                prevTimeRL = currentTime;
                prevCountRL = currentCount;
            } // if (!isMotorRLDone)
            if (!isMotorRRDone) {
                currentCount = rearRight.getCurrentPosition();
                currentTime = ((double) timer.nanoseconds()) * 1.0e-9 - startTime;
                targetCount = getTargetTickCount(tickCount, peakSpeed, rampTime, currentTime);
                currentTargetSpeed = getTargetSpeed(tickCount, peakSpeed, rampTime, currentTime);
                if (initialized) { // check if the motor is rotating
                    isMotorRRNotMoving = Math.abs(currentCount - prevCountRR) < timeOutThreshold;
                }
                if (motorRRForward) {
                    currentError = currentCount - targetCount;
                    if (currentCount >= tickCount) {
                        isMotorRRDone = true;
                        isMotorRRNotMoving = true;
                        currentPower = 0.0;
                        rearRight.setPower(0.0);
                    } else {
                        if (initialized) { // after the first point, the previous data is valid
                            acculErrorRR =
                                    acculErrorRR * alpha
                                            + currentError * (currentTime - prevTimeRR); // integrate error
                            errorSlope = (currentError - prevErrorRR) / (currentTime - prevTimeRR); // error slope
                            currentPower =
                                    currentTargetSpeed / maxSpeed
                                            - currentError * Kp
                                            - acculErrorRR * Ki
                                            - errorSlope * Kd; // apply PID correction
                        } else { // at the first point, use Kp only
                            currentPower = currentTargetSpeed / maxSpeed - currentError * Kp;
                        }
                        if (currentPower > 1.0) currentPower = 1.0;
                        if (currentPower < 0.0) currentPower = 0.0;
                        rearRight.setPower(currentPower);
                    }
                } else { // motorFLForward is false
                    currentError = -currentCount - targetCount;
                    if (currentCount <= -tickCount) {
                        isMotorRRDone = true;
                        isMotorRRNotMoving = true;

                        currentPower = 0.0;
                        rearRight.setPower(0.0);
                    } else {
                        if (initialized) { // after the first point, the previous data is valid
                            acculErrorRR =
                                    acculErrorRR * alpha
                                            + currentError * (currentTime - prevTimeRR); // integrate error
                            errorSlope = (currentError - prevErrorRR) / (currentTime - prevTimeRR); // error slope
                            currentPower =
                                    -currentTargetSpeed / maxSpeed
                                            + currentError * Kp
                                            + acculErrorRR * Ki
                                            + errorSlope * Kd; // apply PID correction
                        } else { // at the first point, use Kp only
                            currentPower = -currentTargetSpeed / maxSpeed + currentError * Kp;
                        }
                        if (currentPower < -1.0) currentPower = -1.0;
                        if (currentPower > 0.0) currentPower = 0.0;
                        rearRight.setPower(currentPower);
                    }
                }
                prevErrorRR = currentError;
                prevTimeRR = currentTime;
                prevCountRR = currentCount;
            } // if (!isMotorRRDone)
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
            String output =
                    String.format(
                            Locale.US,
                            "FL %.1f, %d, FR %.1f %d, RL %.1f %d, RR %.1f %d %.1f %.3f %.1f %.3f %s %s %s %s %s %.1f %s",
                            prevTimeFL * 1000.0,
                            prevCountFL,
                            prevTimeFR * 1000.0,
                            prevCountFR,
                            prevTimeRL * 1000.0,
                            prevCountRL,
                            prevTimeRR * 1000.0,
                            prevCountRR,
                            currentError,
                            acculErrorRR,
                            errorSlope,
                            currentPower,
                            isMotorFLNotMoving ? "Y" : "N",
                            isMotorFRNotMoving ? "Y" : "N",
                            isMotorRLNotMoving ? "Y" : "N",
                            isMotorRRNotMoving ? "Y" : "N",
                            isTimeOutStarted ? "Y" : "N",
                            timeOutStartedTime * 1000.0,
                            isTimeOutExceeded ? "Y" : "N");
            telemetry.addData("motorEnc", output);
            telemetry.update();
        }
    }

    /**
     * Gets the target tick count
     * @param tickCount The tick count
     * @param speed The speed
     * @param rampTime The ramp time
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
            } else { // during ramp down time
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
            } else { // during ramp down time
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
     * @param tickCount the tick count
     * @param speed the speed
     * @param rampTime the ramp time
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
}