package org.firstinspires.ftc.teamcode;

import android.util.Log;
import com.qualcomm.hardware.lynx.LynxModule;
import android.os.Build;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Control;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.Vision;
import org.firstinspires.ftc.teamcode.Util.AllianceColor;

import java.util.List;

public class Robot {
    public final String name = "PowerPlay 2023";

    public final String initLogTag = "init";
    public final ElapsedTime timer;
    // DC Motors
    public DcMotorEx frontLeftDriveMotor;
    public DcMotorEx frontRightDriveMotor;
    public DcMotorEx rearRightDriveMotor;
    public DcMotorEx rearLeftDriveMotor;
    // Odometry
    public List<LynxModule> allHubs;
    public DigitalChannel odometryRA;
    public DigitalChannel odometryRB;
    public DigitalChannel odometryBA;
    public DigitalChannel odometryBB;
    public DigitalChannel odometryLA;
    public DigitalChannel odometryLB;
    public int odRCount = 0;
    public int odBCount = 0;
    public int odLCount = 0;
    /**
     * Declare game pad objects
     */
    public double leftStickX;
    public double leftStickY;
    public double rightStickX;
    public double rightStickY;
    public float triggerLeft;
    public float triggerRight;
    public boolean aButton = false;
    public boolean bButton = false;
    public boolean xButton = false;
    public boolean yButton = false;
    public boolean dPadUp = false;
    public boolean dPadDown = false;
    public boolean dPadLeft = false;
    public boolean dPadRight = false;
    public boolean bumperLeft = false;
    public boolean bumperRight = false;
    public double leftStickX2;
    public double leftStickY2;
    public double rightStickX2;
    public double rightStickY2;
    public float triggerLeft2;
    public float triggerRight2;
    public boolean aButton2 = false;
    public boolean bButton2 = false;
    public boolean xButton2 = false;
    public boolean yButton2 = false;
    public boolean dPadUp2 = false;
    public boolean dPadDown2 = false;
    public boolean dPadLeft2 = false;
    public boolean dPadRight2 = false;
    public boolean bumperLeft2 = false;
    public boolean bumperRight2 = false;
    public boolean isaButtonPressedPrev = false;
    public boolean isbButtonPressedPrev = false;
    public boolean isxButtonPressedPrev = false;
    public boolean isyButtonPressedPrev = false;
    public boolean isdPadUpPressedPrev = false;
    public boolean isdPadDownPressedPrev = false;
    public boolean isdPadLeftPressedPrev = false;
    public boolean isdPadRightPressedPrev = false;
    public boolean islBumperPressedPrev = false;
    public boolean isrBumperPressedPrev = false;
    public boolean isaButton2PressedPrev = false;
    public boolean isbButton2PressedPrev = false;
    public boolean isxButton2PressedPrev = false;
    public boolean isyButton2PressedPrev = false;
    public boolean isdPadUp2PressedPrev = false;
    public boolean isdPadDown2PressedPrev = false;
    public boolean isdPadLeft2PressedPrev = false;
    public boolean isdPadRight2PressedPrev = false;
    public boolean islBumper2PressedPrev = false;
    public boolean isrBumper2PressedPrev = false;
    // Subsystems
    public Drive drive;
    public Control control;
    public Vision vision;
    private final AllianceColor allianceColor;
    private final boolean visionEnabled;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private static final double joystickDeadZone = 0.1;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;

    /**
     * @param timer         The elapsed time
     * @param allianceColor the alliance color
     */
    public Robot(HardwareMap hardwareMap, Telemetry telemetry, ElapsedTime timer, AllianceColor allianceColor, Gamepad gamepad1, Gamepad gamepad2, boolean visionEnabled) {
        Log.i(initLogTag, "started");
        Log.v(initLogTag, "version: " + Build.VERSION.RELEASE);
        this.hardwareMap = hardwareMap;
        this.timer = timer;
        this.allianceColor = allianceColor;
        this.visionEnabled = visionEnabled;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        init();
    }

    public void init() {
        motorInit();
        Log.i(initLogTag, "motor init finished");

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        subsystemInit();
    }

    private void motorInit() {
        frontLeftDriveMotor = (DcMotorEx) hardwareMap.dcMotor.get("fl");
        frontRightDriveMotor = (DcMotorEx) hardwareMap.dcMotor.get("fr");
        rearLeftDriveMotor = (DcMotorEx) hardwareMap.dcMotor.get("bl");
        rearRightDriveMotor = (DcMotorEx) hardwareMap.dcMotor.get("br");


        frontRightDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftDriveMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRightDriveMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rearLeftDriveMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rearRightDriveMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void subsystemInit()
    {
        Log.d(initLogTag, "Drive subsystem init started");
        drive = new Drive(frontLeftDriveMotor, frontRightDriveMotor, rearLeftDriveMotor, rearRightDriveMotor, telemetry, timer);
        Log.i(initLogTag, "Drive subsystem init finished");

        Log.d(initLogTag, "Control subsystem init started");
        control = new Control(telemetry);
        Log.i(initLogTag, "Control subsystem init finished");

        if(visionEnabled) {
            Log.d(initLogTag, "Vision subsystem init started");
            vision = new Vision(telemetry, hardwareMap, allianceColor);
            Log.i(initLogTag, "Vision subsystem init finished");
        }
        else {
            Log.w(initLogTag, "Vision subsystem init skipped");
        }
        telemetryBroadcast("Status", " all subsystems inited");
    }

    public void getGamePadInputs() {
        isaButtonPressedPrev = aButton;
        isbButtonPressedPrev = bButton;
        isxButtonPressedPrev = xButton;
        isyButtonPressedPrev = yButton;
        isdPadUpPressedPrev = dPadUp;
        isdPadDownPressedPrev = dPadDown;
        isdPadLeftPressedPrev = dPadLeft;
        isdPadRightPressedPrev = dPadRight;
        islBumperPressedPrev = bumperLeft;
        isrBumperPressedPrev = bumperRight;
        leftStickX = joystickDeadzoneCorrection(gamepad1.left_stick_x);
        leftStickY = joystickDeadzoneCorrection(-gamepad1.left_stick_y);
        rightStickX = joystickDeadzoneCorrection(gamepad1.right_stick_x);
        rightStickY = joystickDeadzoneCorrection(gamepad1.right_stick_y);
        triggerLeft = gamepad1.left_trigger;
        triggerRight = gamepad1.right_trigger;
        aButton = gamepad1.a;
        bButton = gamepad1.b;
        xButton = gamepad1.x;
        yButton = gamepad1.y;
        dPadUp = gamepad1.dpad_up;
        dPadDown = gamepad1.dpad_down;
        dPadLeft = gamepad1.dpad_left;
        dPadRight = gamepad1.dpad_right;
        bumperLeft = gamepad1.left_bumper;
        bumperRight = gamepad1.right_bumper;

        isaButton2PressedPrev = aButton2;
        isbButton2PressedPrev = bButton2;
        isxButton2PressedPrev = xButton2;
        isyButton2PressedPrev = yButton2;
        isdPadUp2PressedPrev = dPadUp2;
        isdPadDown2PressedPrev = dPadDown2;
        isdPadLeft2PressedPrev = dPadLeft2;
        isdPadRight2PressedPrev = dPadRight2;
        islBumper2PressedPrev = bumperLeft2;
        isrBumper2PressedPrev = bumperRight2;
        leftStickX2 = joystickDeadzoneCorrection(gamepad2.left_stick_x);
        leftStickY2 = joystickDeadzoneCorrection(-gamepad2.left_stick_y);
        rightStickX2 = joystickDeadzoneCorrection(gamepad2.right_stick_x);
        rightStickY2 = joystickDeadzoneCorrection(-gamepad2.right_stick_y);
        triggerLeft2 = gamepad2.left_trigger;
        triggerRight2 = gamepad2.right_trigger;
        aButton2 = gamepad2.a;
        bButton2 = gamepad2.b;
        xButton2 = gamepad2.x;
        yButton2 = gamepad2.y;
        dPadUp2 = gamepad2.dpad_up;
        dPadDown2 = gamepad2.dpad_down;
        dPadLeft2 = gamepad2.dpad_left;
        dPadRight2 = gamepad2.dpad_right;
        bumperLeft2 = gamepad2.left_bumper;
        bumperRight2 = gamepad2.right_bumper;
    }

    /**
     * Discards joystick inputs between -joystickDeadZone and joystickDeadZone
     * @param joystickInput the input of the joystick
     * @return the corrected input of the joystick
     */
    public double joystickDeadzoneCorrection(double joystickInput) {
        double joystickOutput;
        if (joystickInput > joystickDeadZone) {
            joystickOutput = (joystickInput - joystickDeadZone) / (1.0 - joystickDeadZone);
        } else if (joystickInput > -joystickDeadZone) {
            joystickOutput = 0.0;
        } else {
            joystickOutput = (joystickInput + joystickDeadZone) / (1.0 - joystickDeadZone);
        }
        return joystickOutput;
    }

    public void telemetryBroadcast(String caption, String value) {
        telemetry.addData(caption, value);
        telemetry.update();
    }
}
