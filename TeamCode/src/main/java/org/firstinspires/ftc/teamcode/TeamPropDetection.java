package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ControlAlgorithms.PIDController;
import org.firstinspires.ftc.teamcode.Drivetrain.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrivetrain;

@TeleOp(name = "Team Prop Detection")
public class TeamPropDetection extends LinearOpMode {
    private MecanumDrivetrain drivetrain;
    private DistanceSensor sensorDistance;
    private IMU imu;
    private final double STARTING_DISTANCE = 35;
    private double currentHeading;
    private static final double MAX_SPEED = 0.8;
    private static final double DETECT_RANGE = 10.0;
    @Override
    public void runOpMode() {
        /* initialize IMU */
        imu = hardwareMap.get(IMU.class, RobotConfig.IMU_NAME);
        IMU.Parameters imuParams = new IMU.Parameters(new RevHubOrientationOnRobot(RobotConfig.CH_LOGO_DIRECTION, RobotConfig.CH_USB_DIRECTION));
        imu.initialize(imuParams);
        imu.resetYaw(); // heading

        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensorDistance");

        /* Run to Spike Mark */
        drivetrain = new MecanumDrivetrain(hardwareMap, telemetry);
        PIDController driveController = new PIDController(DrivetrainConstants.DRIVE_Y_PID, -MAX_SPEED, MAX_SPEED);
        double y = driveController.control(STARTING_DISTANCE);
        drivetrain.drive(0,y,0);

        /* Rotate continuously to find prop */
        while (!Detected()) {
            drivetrain.turnBy(1);
            currentHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            telemetry.addData("Current Heading:","%.3f deg", currentHeading);
            telemetry.update();
        }
        currentHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        telemetry.addData("Detected Heading:","%.3f deg", currentHeading);
    }
    private boolean Detected() {
        double distance = sensorDistance.getDistance(DistanceUnit.INCH);
        return distance <= DETECT_RANGE;
    }
}
