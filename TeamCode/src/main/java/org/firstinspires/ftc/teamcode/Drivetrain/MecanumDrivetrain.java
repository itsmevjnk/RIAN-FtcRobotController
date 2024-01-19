package org.firstinspires.ftc.teamcode.Drivetrain;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ControlAlgorithms.PIDController;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.Drivetrain.DrivetrainConstants;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumDrivetrain {
    private DcMotorEx driveLeftBack, driveRightBack, driveLeftFront, driveRightFront;
    private IMU imu;

    private Telemetry telemetry;
    private ElapsedTime time;

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

        /* initialize IMU */
        imu = hardwareMap.get(IMU.class, RobotConfig.IMU_NAME);
        IMU.Parameters imuParams = new IMU.Parameters(new RevHubOrientationOnRobot(RobotConfig.CH_LOGO_DIRECTION, RobotConfig.CH_USB_DIRECTION));
        imu.initialize(imuParams);
        imu.resetYaw(); // heading

        time = new ElapsedTime();
        lastUpdateTime = time.seconds();

        brake(); // reset driving and stop/brake entire drivetrain
    }

    /* drivetrain controlling given X/Y translation and rotation speeds */
//    public double lastX = 0.0, lastY = 0.0, lastRotation = 0.0; // last input values (-1.0 .. 1.0)
    public void drive(double x, double y, double rot) {
//        lastX = x; lastY = y; lastRotation = rot;

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

    /* odometry */
    public double posX = 0, posY = 0, velX = 0, velY = 0, posHeading = 0;
    public double lastUpdateTime = 0;

    public void resetPosition() {
        imu.resetYaw();
        posX = 0; posY = 0; posHeading = 0;
        lastUpdateTime = time.seconds();
    }

    public void updatePosition() { // to be called on loop
        /* calculate robot-centric velocity (in ticks per second) */
        double vy = (driveLeftFront.getVelocity() + driveRightFront.getVelocity() + driveLeftBack.getVelocity() + driveRightBack.getVelocity()) / 4.0;
        double vx = (driveLeftFront.getVelocity() - driveRightFront.getVelocity() - driveLeftBack.getVelocity() + driveRightBack.getVelocity()) / 4.0;

        /* update robot heading */
        posHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double posHeadingRadians = Math.toRadians(posHeading);

        /* rotate robot-centric velocity to field-centric and convert to inch/sec */
        double factor = (1 / DrivetrainConstants.DRIVE_ENCODER_RESOLUTION) * Math.PI * DrivetrainConstants.WHEEL_DIAMETER;
        velX = (vx * Math.cos(-posHeadingRadians) - vy * Math.sin(-posHeadingRadians)) * factor;
        velY = (vx * Math.sin(-posHeadingRadians) + vy * Math.cos(-posHeadingRadians)) * factor;

        /* integrate velocity over time */
        double currentTime = time.seconds(); double dt = currentTime - lastUpdateTime; lastUpdateTime = currentTime;
        posX += velX * dt;
        posY += velY * dt;

        telemetry.addData("Pose", "x: %.3f in, y: %.3f in, hdg: %.3f deg", posX, posY, posHeading);
        telemetry.addData("Velocity", "x: %.3f in/s, y: %.3f in/s", velX, velY);
    }

    /* PID */
    public PIDController pidX = new PIDController(DrivetrainConstants.DRIVE_X_PID, -DrivetrainConstants.DRIVE_X_AUTO_MAX, DrivetrainConstants.DRIVE_X_AUTO_MAX);
    public PIDController pidY = new PIDController(DrivetrainConstants.DRIVE_Y_PID, -DrivetrainConstants.DRIVE_Y_AUTO_MAX, DrivetrainConstants.DRIVE_Y_AUTO_MAX);
    public PIDController pidHeading = new PIDController(DrivetrainConstants.DRIVE_ROT_PID, -DrivetrainConstants.DRIVE_ROT_AUTO_MAX, DrivetrainConstants.DRIVE_ROT_AUTO_MAX);

    public void resetPID() { // must be called prior to any driving action
        pidX.reset();
        pidY.reset();
        pidHeading.reset();
    }

    public boolean driveTo(double x, double y) { // returns true on reaching destination
        updatePosition(); // get current position
        double errX = posX - x, errY = posY - y;

        /* rotate position error to robot-centric */
        double posHeadingRadians = Math.toRadians(posHeading);
        double errXRobot = errX * Math.cos(-posHeadingRadians) - errY * Math.sin(-posHeadingRadians);
        double errYRobot = errX * Math.sin(-posHeadingRadians) + errY * Math.cos(-posHeadingRadians);

        telemetry.addData("Error", "x: %.3f (%.3f) in, y: %.3f (%.3f) in", errX, errXRobot, errY, errYRobot);

        /* drive/strafe to specified X and Y */
        if(Math.abs(errXRobot) > DrivetrainConstants.DRIVE_X_AUTO_TOLERANCE || Math.abs(errYRobot) > DrivetrainConstants.DRIVE_Y_AUTO_TOLERANCE) {
            drive(pidX.control(errXRobot), pidY.control(errYRobot), 0);
            return false;
        }

        return true; // we're all set
    }

    public void driveBy(double dx, double dy) {
        double targetX = posX + dx, targetY = posY + dy;
        while(!driveTo(targetX, targetY)) telemetry.update();
    }

    public boolean turnTo(double hdg) { // returns true on reaching destination
        hdg = AngleUnit.normalizeDegrees(hdg); // normalize desired heading to -180...180

        updatePosition(); // get current position
        double errHeading = AngleUnit.normalizeDegrees(posHeading - hdg);

        telemetry.addData("Error", "%.3f deg", errHeading);

        /* drive/strafe to specified X and Y */
        if(Math.abs(errHeading) > DrivetrainConstants.DRIVE_ROT_AUTO_TOLERANCE) {
            drive(0, 0, -pidHeading.control(errHeading));
            return false;
        }

        return true; // we're all set
    }

    public void turnBy(double dhdg) {
        double targetHeading = posHeading + dhdg;
        while(!turnTo(targetHeading)) telemetry.update();
    }
}
