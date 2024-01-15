package org.firstinspires.ftc.teamcode.Drivetrain;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class DrivetrainConstants {
    /* drivetrain parameters - in milimeters */
    public static final double TRACK_WIDTH = 405.0;
    public static final double WHEEL_DIAMETER = 96.0;

    /* driving speed multipliers */
    public static final double DRIVE_Y_RATIO = 1.0;
    public static final double DRIVE_X_RATIO = 1.1; // for counteracting imperfect strafing
    public static final double DRIVE_ROT_RATIO = 1.0;
}
