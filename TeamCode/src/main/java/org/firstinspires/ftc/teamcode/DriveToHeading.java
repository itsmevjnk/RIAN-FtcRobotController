package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ControlAlgorithms.PIDController;
import org.firstinspires.ftc.teamcode.Drivetrain.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrivetrain;

@Autonomous(name = "Drive to heading")
public class DriveToHeading extends LinearOpMode {
    private MecanumDrivetrain drivetrain;

    private static final double MAX_AUTO_TURN = 1.00;
    private PIDController turnController = new PIDController(DrivetrainConstants.DRIVE_ROT_PID, -MAX_AUTO_TURN, MAX_AUTO_TURN);

    /* heading info in degrees */
    private static final double DESIRED_HEADING = 180.0;
    private static final double HEADING_ERR_TOLERANCE = 0.25;

    private IMU imu;

    @Override
    public void runOpMode() {
        drivetrain = new MecanumDrivetrain(hardwareMap, telemetry);

        imu = hardwareMap.get(IMU.class, RobotConfig.IMU_NAME);
        IMU.Parameters imuParams = new IMU.Parameters(new RevHubOrientationOnRobot(RobotConfig.CH_LOGO_DIRECTION, RobotConfig.CH_USB_DIRECTION));
        imu.initialize(imuParams);

        waitForStart();

        imu.resetYaw();

        while(!isStopRequested()) {
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = currentHeading - DESIRED_HEADING;
            double rot = turnController.control(error); // NOTE: robot heading's (+) direction is counterclockwise, while the (+) direction for drivetrain rotation is clockwise!

            telemetry.addData("currentHeading", currentHeading);
            telemetry.addData("error", error);
            telemetry.addData("rot", rot);
            telemetry.update();

            if(Math.abs(error) <= HEADING_ERR_TOLERANCE) break; // stop if we are within tolerance
            drivetrain.drive(0, 0, rot);
        }
    }
}
