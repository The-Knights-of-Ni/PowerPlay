package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.Drive;
import org.firstinspires.ftc.teamcode.Util.Vector;
import org.junit.jupiter.api.Test;

class DriveTest {

    @Test
    void testCalcMotorPower2D() {
        MockDcMotorEx mockFL = new MockDcMotorEx(DcMotor.RunMode.RUN_USING_ENCODER);
        MockDcMotorEx mockFR = new MockDcMotorEx(DcMotor.RunMode.RUN_USING_ENCODER);
        MockDcMotorEx mockRL = new MockDcMotorEx(DcMotor.RunMode.RUN_USING_ENCODER);
        MockDcMotorEx mockRR = new MockDcMotorEx(DcMotor.RunMode.RUN_USING_ENCODER);
        ElapsedTime timer = new ElapsedTime();
        MockTelemetry telemetry = new MockTelemetry();
        Drive drive = new Drive(mockFL, mockFR, mockRL, mockRR, telemetry, timer);
        assertEquals(0.5, drive.calcMotorPowers(0, 1, 0)[0], 0.5);
        assertEquals(0.5, drive.calcMotorPowers(0, 1, 0)[1], 0.5);
        assertEquals(0.5, drive.calcMotorPowers(0, 1, 0)[2], 0.5);
        assertEquals(0.5, drive.calcMotorPowers(0, 1, 0)[3], 0.5);
        assertNotEquals(0, drive.calcMotorPowers(0, 1, 0)[0]);
        assertNotEquals(0, drive.calcMotorPowers(0, 1, 0)[1]);
        assertNotEquals(0, drive.calcMotorPowers(0, 1, 0)[2]);
        assertNotEquals(0, drive.calcMotorPowers(0, 1, 0)[3]);
    }

    @Test
    void testMoveVector2D() {
        MockDcMotorEx mockFL = new MockDcMotorEx(DcMotor.RunMode.RUN_USING_ENCODER);
        MockDcMotorEx mockFR = new MockDcMotorEx(DcMotor.RunMode.RUN_USING_ENCODER);
        MockDcMotorEx mockRL = new MockDcMotorEx(DcMotor.RunMode.RUN_USING_ENCODER);
        MockDcMotorEx mockRR = new MockDcMotorEx(DcMotor.RunMode.RUN_USING_ENCODER);
        ElapsedTime timer = new ElapsedTime();
        MockTelemetry telemetry = new MockTelemetry();
        Drive drive = new Drive(mockFL, mockFR, mockRL, mockRR, telemetry, timer);
        drive.moveVector(new Vector(1000,1000));
        assertEquals(0, drive.frontLeft.getPower(), 0.0001);
        assertEquals(0, drive.frontRight.getPower(), 0.0001);
        assertEquals(0, drive.rearLeft.getPower(), 0.0001);
        assertEquals(0, drive.rearRight.getPower(), 0.0001);
    }
}

