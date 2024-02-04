package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
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
    private MecanumDrive drivetrain;
    private DistanceSensor sensorDistance;
    private IMU imu;
    private final double STARTING_DISTANCE = 35;
    private double currentHeading;
    private static final double DETECT_RANGE = 10.0;
    private final double SCAN_ANGLE = 20;
    private final double MAX_TURNING_SPEED = 0.5;
    private PIDController turnController;
    @Override
    public void runOpMode() {
        /* initialize IMU */
        imu = hardwareMap.get(IMU.class, RobotConfig.IMU_NAME);
        IMU.Parameters imuParams = new IMU.Parameters(new RevHubOrientationOnRobot(RobotConfig.CH_LOGO_DIRECTION, RobotConfig.CH_USB_DIRECTION));
        imu.initialize(imuParams);
        imu.resetYaw(); // heading

        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensorDistance");

        turnController = new PIDController(DrivetrainConstants.DRIVE_ROT_PID, -MAX_TURNING_SPEED, MAX_TURNING_SPEED);

        /* Run to Spike Mark */
        drivetrain = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        Vector2d destCord = new Vector2d(
                STARTING_DISTANCE,
                0
        );
        Actions.runBlocking(drivetrain.actionBuilder(drivetrain.pose).strafeToConstantHeading(destCord).build());

        if (!Detected()) {
            Actions.runBlocking(drivetrain.actionBuilder(drivetrain.pose).turn(-80).build());
            if (!Scan(-SCAN_ANGLE)) {
                Actions.runBlocking(drivetrain.actionBuilder(drivetrain.pose).turn(180).build());
                Scan(SCAN_ANGLE);
            }
        }
        currentHeading = Math.toDegrees(drivetrain.pose.heading.toDouble());
        telemetry.addData("Detected Heading:","%.3f deg", currentHeading);
        telemetry.update();
    }
    private boolean Scan(double angle) {
        if (angle < 0) {
            double rot = turnController.control(1);
            while (angle ++ < 0) {
                drivetrain.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                0,0
                        ),
                        rot
                ));
                if (Detected()) return true;
            }
        } else {
            double rot = turnController.control(-1);
            while (angle -- > 0) {
                drivetrain.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                0,0
                        ),
                        rot
                ));
                if (Detected()) return true;
            }
        }
        return false;
    }
    private boolean Detected() {
        double distance = sensorDistance.getDistance(DistanceUnit.INCH);
        return distance <= DETECT_RANGE;
    }
}
