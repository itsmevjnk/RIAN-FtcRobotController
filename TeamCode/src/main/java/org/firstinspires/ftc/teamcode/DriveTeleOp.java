package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.Mechanisms.Slide;

@TeleOp(name = "Driving TeleOp")
public class DriveTeleOp extends LinearOpMode {
    private MecanumDrive drivetrain;

    private Launcher launcher;
    private Slide slide;
    private Intake intake;
    private DistanceSensor sensorDistance;
    private final double STOPPING_DISTANCE = 2; //inch
    private boolean lastState;
    private boolean currentSlideState;

    @Override
    public void runOpMode() {
        drivetrain = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        launcher = new Launcher(hardwareMap);
        slide = new Slide(hardwareMap);
        intake = new Intake(hardwareMap);
        sensorDistance = hardwareMap.get(DistanceSensor.class,"sensorDistance");
        lastState = currentSlideState = false; //retracted

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            double x = -gamepad1.left_stick_x;

            if (sensorDistance.getDistance(DistanceUnit.INCH) <= STOPPING_DISTANCE && x > 0) x = 0;

            drivetrain.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            x
                    ),
                    -gamepad1.right_stick_x
            ));
            drivetrain.updatePoseEstimate();

            telemetry.addData("x", drivetrain.pose.position.x);
            telemetry.addData("y", drivetrain.pose.position.y);
            telemetry.addData("Heading", Math.toDegrees(drivetrain.pose.heading.toDouble()));

            if (gamepad1.cross && !lastState) currentSlideState = !currentSlideState;
            lastState = gamepad1.cross;
            if (gamepad1.right_bumper) intake.setMotor(2);
            else if (gamepad1.left_bumper) intake.setMotor(1);
            else intake.setMotor(0);
            launcher.setState(gamepad1.options);
            slide.setState(currentSlideState);
            telemetry.update();
        }
    }
}
