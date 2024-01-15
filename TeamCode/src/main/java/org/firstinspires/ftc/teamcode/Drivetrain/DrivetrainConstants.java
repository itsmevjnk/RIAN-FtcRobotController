package org.firstinspires.ftc.teamcode.Drivetrain;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class DrivetrainConstants {
    /* drivetrain motor names */
    public static final String LEFT_BACK_NAME = "driveLeftBack";
    public static final String RIGHT_BACK_NAME = "driveRightBack";
    public static final String LEFT_FRONT_NAME = "driveLeftFront";
    public static final String RIGHT_FRONT_NAME = "driveRightFront";

    /* drivetrain parameters - in milimeters */
    public static final double TRACK_WIDTH = 405.0;
    public static final double WHEEL_DIAMETER = 96.0;

    /* driving speed multipliers */
    public static final double DRIVE_Y_RATIO = 1.0;
    public static final double DRIVE_X_RATIO = 1.1; // for counteracting imperfect strafing
    public static final double DRIVE_ROT_RATIO = 1.0;

    /* Control Hub/IMU configuration */
    public static final String IMU_NAME = "imu"; // default name
    public static RevHubOrientationOnRobot.LogoFacingDirection CH_LOGO_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection CH_USB_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
}
