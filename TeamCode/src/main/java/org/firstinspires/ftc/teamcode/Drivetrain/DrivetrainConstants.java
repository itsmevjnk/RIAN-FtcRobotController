package org.firstinspires.ftc.teamcode.Drivetrain;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class DrivetrainConstants {
    /* drivetrain parameters - in inches */
    public static final double TRACK_WIDTH = 405.0 / 25.4;
    public static final double WHEEL_DIAMETER = 96.0 / 25.4;
    public static final double DRIVE_ENCODER_RESOLUTION = ((((1+(46.0/17))) * (1+(46.0/11))) * 28);

    /* driving speed multipliers */
    public static final double DRIVE_Y_RATIO = 1.0;
    public static final double DRIVE_X_RATIO = 1.1; // for counteracting imperfect strafing
    public static final double DRIVE_ROT_RATIO = 1.0;

    /* autonomous PID coefficients */
    public static PIDCoefficients DRIVE_X_PID = new PIDCoefficients(0.02, 0.0, 0.0);
    public static PIDCoefficients DRIVE_Y_PID = new PIDCoefficients(0.02, 0.0, 0.0);
    public static PIDCoefficients DRIVE_ROT_PID = new PIDCoefficients(0.01, 0.0, 0.000001);

    /* autonomous maximum speed */
    public static final double DRIVE_X_AUTO_MAX = 1.0;
    public static final double DRIVE_Y_AUTO_MAX = 1.0;
    public static final double DRIVE_ROT_AUTO_MAX = 1.0;

    /* autonomous error tolerance */
    public static final double DRIVE_X_AUTO_TOLERANCE = 0.15;
    public static final double DRIVE_Y_AUTO_TOLERANCE = 0.15;
    public static final double DRIVE_ROT_AUTO_TOLERANCE = 0.25;
}
