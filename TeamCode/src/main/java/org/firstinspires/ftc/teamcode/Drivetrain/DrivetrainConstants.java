package org.firstinspires.ftc.teamcode.Drivetrain;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class DrivetrainConstants {
    /* drivetrain parameters - in milimeters */
    public static final double TRACK_WIDTH = 405.0;
    public static final double WHEEL_DIAMETER = 96.0;

    /* driving speed multipliers */
    public static final double DRIVE_Y_RATIO = 1.0;
    public static final double DRIVE_X_RATIO = 1.1; // for counteracting imperfect strafing
    public static final double DRIVE_ROT_RATIO = 1.0;

    public static PIDCoefficients DRIVE_X_PID = new PIDCoefficients(0.005, 0.0, 0.000005);
    public static PIDCoefficients DRIVE_Y_PID = new PIDCoefficients(0.02, 0.0, 0.0);
    public static PIDCoefficients DRIVE_ROT_PID = new PIDCoefficients(0.005, 0.0, 0.000001);
}
