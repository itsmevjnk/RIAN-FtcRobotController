package org.firstinspires.ftc.teamcode.Drivetrain;

import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.Drivetrain.DrivetrainConstants;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumDrivetrain {
    private DcMotorEx driveLeftBack, driveRightBack, driveLeftFront, driveRightFront;
    private Telemetry telemetry;

    public MecanumDrivetrain(HardwareMap hardwareMap, Telemetry tele) {
        telemetry = tele; // save telemetry instance for later

        /* get drivetrain motor instances */
        driveLeftBack = hardwareMap.get(DcMotorEx.class, RobotConfig.DRIVE_LEFT_BACK);
        driveRightBack = hardwareMap.get(DcMotorEx.class, RobotConfig.DRIVE_RIGHT_BACK);
        driveLeftFront = hardwareMap.get(DcMotorEx.class, RobotConfig.DRIVE_LEFT_FRONT);
        driveRightFront = hardwareMap.get(DcMotorEx.class, RobotConfig.DRIVE_RIGHT_FRONT);

        /* reverse direction on the left motors */
        driveLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        driveLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        /* set all drivetrain motors to run using encoder and brake on zero power */
        driveLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        brake(); // reset driving and stop/brake entire drivetrain
    }

    /* drivetrain controlling given X/Y translation and rotation speeds */
    public double lastX = 0.0, lastY = 0.0, lastRotation = 0.0; // last input values (-1.0 .. 1.0)
    public void drive(double x, double y, double rot) {
        lastX = x; lastY = y; lastRotation = rot;

        x *= DrivetrainConstants.DRIVE_X_RATIO; y *= DrivetrainConstants.DRIVE_Y_RATIO; rot *= DrivetrainConstants.DRIVE_ROT_RATIO;

        double denom = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rot), 1.0); // denominator (max 1.0)
        double lbSpeed = (y - x + rot) / denom; driveLeftBack.setPower(lbSpeed);
        double lfSpeed = (y + x + rot) / denom; driveLeftFront.setPower(lfSpeed);
        double rbSpeed = (y + x - rot) / denom; driveRightBack.setPower(rbSpeed);
        double rfSpeed = (y - x - rot) / denom; driveRightFront.setPower(rfSpeed);
        telemetry.addData("Motor speed", "lb: %.3f, lf: %.3f, rb: %.3f, rf: %.3f", lbSpeed, lfSpeed, rbSpeed, rfSpeed);
    }

    public void brake() {
        drive(0, 0, 0);
    }
}
