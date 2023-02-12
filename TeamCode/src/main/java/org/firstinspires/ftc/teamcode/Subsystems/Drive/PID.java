package org.firstinspires.ftc.teamcode.Subsystems.Drive;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID implements ControlSystem {
    private final double Kp;
    private final double Ki;
    private final double Kd;

    protected boolean hasRun = false;

    protected ElapsedTime timer = new ElapsedTime();

    protected double previousError = 0;
    protected double integralSum = 0;
    protected double derivative = 0;

    public PID(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    /**
     * calculate PID output
     * @param target the target position
     * @param measured current system state
     * @return PID output (a double from -1 to 1)
     */
    @Override
    public double calculate(double target, double measured) {
        double dt = getDT();
        double error = calculateError(target, measured);
        double derivative = calculateDerivative(error, dt);
        integrate(error, dt);
        previousError = error;
        // Cap output at range (-1,1)
        return Math.min(1, Math.max(-1, error * Kp + integralSum * Ki + derivative * Kd));
    }

    /**
     * get the time constant
     * @return time constant
     */
    public double getDT() {
        if (!hasRun) {
            hasRun = true;
            timer.reset();
        }
        double dt = timer.seconds();
        timer.reset();
        return dt;
    }

    protected double calculateError(double target, double measured) {
        return target - measured;
    }

    protected void integrate(double error, double dt) {
        integralSum += ((error + previousError) / 2) * dt;
    }

    protected double calculateDerivative(double error, double dt) {
        derivative = (error - previousError) / dt;
        return derivative;
    }
}