package org.firstinspires.ftc.teamcode.ControlAlgorithms;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PIDController {
    private PIDCoefficients coefficients; // PID coefficients (kP, kI, kD)
    private ElapsedTime time;
    private double clipMin = Double.NEGATIVE_INFINITY;
    private double clipMax = Double.POSITIVE_INFINITY;

    private double accError = 0; // accumulated error over time
    private double lastError = 0; // last error
    private double lastControlTimestamp = 0; // last control timestamp
    private boolean first = true; // cleared upon first control after initialization/reset

    public PIDController(PIDCoefficients coeff, double min, double max) {
        coefficients = coeff;
        time = new ElapsedTime();
        clipMin = min; clipMax = max;
    }

    public PIDController(PIDCoefficients coeff) {
        this(coeff, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    public void reset() {
        accError = 0;
        first = true;
    }

    public double control(double error) {
        double timestamp = time.seconds();
        double cv = coefficients.p * error; // control variable (P only for now)

        if(!first) {
            /* subsequent runs here - accumulate errors over time */
            double dt = timestamp - lastControlTimestamp; // duration since last control action
            accError += error * dt; // accumulate error
            cv += coefficients.i * accError + coefficients.d * (error - lastError) / dt; // calculate I and D
        }
        first = false; // next run will no longer be the first run

        lastError = error; // for next run
        lastControlTimestamp = timestamp;
        return Range.clip(cv, clipMin, clipMax);
    }
}
