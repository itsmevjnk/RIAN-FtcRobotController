package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ControlAlgorithms.PIDController;
import org.firstinspires.ftc.teamcode.Drivetrain.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrivetrain;

@Autonomous(name = "Drive to position")
public class DriveToPosition extends LinearOpMode {
    private MecanumDrivetrain drivetrain;

    private static final double MAX_AUTO_TURN = 1.00;
    private PIDController turnController = new PIDController(DrivetrainConstants.DRIVE_ROT_PID, -MAX_AUTO_TURN, MAX_AUTO_TURN);

    /* position info */
    private static final double DESIRED_X = 10.0;
    private static final double DESIRED_Y = 10.0;
    private static final double DESIRED_HEADING = 90.0; // TODO: driveTrain.driveTo() can't handle 180-degree headings

    private IMU imu;

    @Override
    public void runOpMode() {
        drivetrain = new MecanumDrivetrain(hardwareMap, telemetry);

        imu = hardwareMap.get(IMU.class, RobotConfig.IMU_NAME);
        IMU.Parameters imuParams = new IMU.Parameters(new RevHubOrientationOnRobot(RobotConfig.CH_LOGO_DIRECTION, RobotConfig.CH_USB_DIRECTION));
        imu.initialize(imuParams);

        waitForStart();

        drivetrain.resetPosition();
        boolean doneHeading = false; // set when heading is OK
        while(opModeIsActive()) {
            if(doneHeading) {
                /* turn first, then drive */
                if(drivetrain.driveTo(DESIRED_X, DESIRED_Y)) return;
            } else doneHeading = drivetrain.turnTo(DESIRED_HEADING);
            telemetry.update();
        }

    }
}
