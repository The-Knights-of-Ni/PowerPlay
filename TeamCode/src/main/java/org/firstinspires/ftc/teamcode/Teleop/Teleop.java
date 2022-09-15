package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Control;
import org.firstinspires.ftc.teamcode.Subsystems.Control.BucketState;
import org.firstinspires.ftc.teamcode.Subsystems.Control.SlideState;
import org.firstinspires.ftc.teamcode.Subsystems.Control.LidPosition;
import org.firstinspires.ftc.teamcode.Subsystems.Control.MarkerHookState;
import org.firstinspires.ftc.teamcode.Util.AllianceColor;

import java.io.IOException;

@TeleOp(name = "Teleop")
public class Teleop extends LinearOpMode {
    double deltaT;
    double timeCurrent;
    double timePre;
    ElapsedTime timer;
    private Robot robot;
    private double robotAngle;

    private boolean driveHighPower = true;

    private boolean isIntakeOn = false;
    private boolean isDuckOn = false;
    private boolean isBucketLevel = false;
    private boolean isSlideUp = false;
    private boolean scoreTop = true; //bottom is false
    private final int slowModePow = 5;

    private void initOpMode() throws IOException {
        // Initialize DC motor objects
        timer = new ElapsedTime();
        this.robot =  new Robot(this, hardwareMap, telemetry, timer, AllianceColor.BLUE, gamepad1, gamepad2,false);

        timeCurrent = timer.nanoseconds();
        timePre = timeCurrent;

        telemetry.addData("Waiting for start", "...");
        telemetry.update();
    }

    private void initDevices() {
        robot.control.setBucketState(BucketState.LEVEL);
        isBucketLevel = true;
        robot.control.setLidPosition(LidPosition.OPEN);
    }

    /**
     * Override of runOpMode()
     *
     * <p>Please do not swallow the InterruptedException, as it is used in cases where the op mode
     * needs to be terminated early.
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
     */
    @Override
    public void runOpMode() throws InterruptedException {
        try {
            initOpMode();
        } catch (IOException e) {
            e.printStackTrace();
        }

        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        initDevices();

        if (isStopRequested()) {
            return;
        }

        telemetry.clearAll();
        timeCurrent = timer.nanoseconds();
        timePre = timeCurrent;

        while (opModeIsActive()) { // clearer nomenclature for variables
            robot.getGamePadInputs();

            timeCurrent = timer.nanoseconds();
            deltaT = timeCurrent - timePre;
            timePre = timeCurrent;

            // Robot drive movement
            double[] motorPowers;
            robotAngle = robot.imu.getAngularOrientation().firstAngle;
            if (driveHighPower) {
                motorPowers = robot.drive.calcMotorPowers(robot.leftStickX, robot.leftStickY, robot.rightStickX);
            }
            else {
                motorPowers = robot.drive.calcMotorPowers(robot.leftStickX*0.5, robot.leftStickY*0.5, robot.rightStickX*0.5);
            }
            robot.drive.setDrivePowers(motorPowers);

            // Toggle intake regular
            if (robot.aButton && !robot.isaButtonPressedPrev) {
                if (isIntakeOn) {
                    robot.control.setIntakeDirection(false, true);
                    isIntakeOn = false;
                } else {
                    robot.control.setIntakeDirection(true, true);
                    isIntakeOn = true;
                }
            }

            // Toggle intake reverse
            if (robot.bButton && !robot.isbButtonPressedPrev) {
                if (isIntakeOn) {
                    robot.control.setIntakeDirection(false, false);
                    isIntakeOn = false;
                } else {
                    robot.control.setIntakeDirection(true, false);
                    isIntakeOn = true;
                }
            }

            /**
             * Intake only moves here to prevent bucket from being stuck, use buttons A and B to control the intake.
             */
            // Toggle bucket up-level
            if (robot.xButton && !robot.isxButtonPressedPrev) {
                if (isBucketLevel) {
                    robot.control.setIntakeDirection(true, true);
                    robot.control.setBucketState(BucketState.RAISED);
                    isIntakeOn = true;
                    isBucketLevel = false;
                } else {
                    robot.control.setIntakeDirection(true, false);
                    robot.control.setBucketState(BucketState.LEVEL);
                    isIntakeOn = true;
                    isBucketLevel = true;
                }
            }

            //Toggle drive power
            if (robot.yButton && !robot.isyButtonPressedPrev){
                driveHighPower = !driveHighPower;
            }
            
            //Toggle scoring level - toggle when with alliance
            if (robot.bumperLeft && !robot.islBumperPressedPrev) {
                scoreTop = !scoreTop;
            }

            // Toggle slide up
            if (robot.aButton2 && !robot.isaButton2PressedPrev) {
                if(!isBucketLevel) {
                    robot.control.setIntakeDirection(true, false);
                    robot.control.setBucketState(BucketState.LEVEL);
                    isBucketLevel = true;
                }
                robot.control.setLidPosition(LidPosition.CLOSED);
                if (scoreTop) {
                    robot.control.setSlide(SlideState.TOP);
                } else {
                    robot.control.setSlide(SlideState.BOTTOM);
                }
                
                isSlideUp = true;
            }

            // Toggle slide retracted
            if (robot.bButton2 && !robot.isbButton2PressedPrev) {
                // Set bucket level first if it's not
                if(!isBucketLevel) {
                    robot.control.setIntakeDirection(true, false);
                    isIntakeOn = true;
                    robot.control.setBucketState(BucketState.LEVEL);
                    isBucketLevel = true;
                }

                robot.control.setLidPosition(LidPosition.CLOSED);
                robot.control.setSlide(SlideState.RETRACTED);
                isSlideUp = false;
            }
            if(robot.control.isSlideRetracted() && !isSlideUp) {
                    robot.control.setLidPosition(LidPosition.OPEN);
            } else {
                // Toggle lid deployed/closed
                if(robot.bumperLeft2 && !robot.islBumper2PressedPrev && isSlideUp) {
                    robot.control.setLidPosition(LidPosition.CLOSED);
                }
                if(robot.bumperRight2 && !robot.isrBumper2PressedPrev && isSlideUp) {
                    robot.control.setLidPosition(LidPosition.DEPLOYED);
                }
            }

            // Toggle duck wheel forward
            if (robot.xButton2 && !robot.isxButton2PressedPrev) {
                if (isDuckOn) {
                    robot.control.stopCarousel();
                    isDuckOn = false;
                } else {
                    robot.control.startCarousel(true);
                    isDuckOn = true;
                }
            }

            // Toggle duck wheel reverse
            if (robot.yButton2 && !robot.isyButton2PressedPrev) {
                if (isDuckOn) {
                    robot.control.stopCarousel();
                    isDuckOn = false;
                } else {
                    robot.control.startCarousel(false);
                    isDuckOn = true;
                }
            }

            //Marker claw/hook toggle up/down
            if(robot.triggerRight2 >= 0.5) {
                robot.control.runMarkerHook(MarkerHookState.UP);
            }
            if(robot.triggerLeft2 >= 0.5) {
                robot.control.runMarkerHook(MarkerHookState.DOWN);
            }
        }
    }
}
