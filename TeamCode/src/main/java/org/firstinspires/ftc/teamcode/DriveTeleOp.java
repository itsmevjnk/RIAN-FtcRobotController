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
import org.firstinspires.ftc.teamcode.Mechanisms.Outtake;
import org.firstinspires.ftc.teamcode.Mechanisms.Slide;

@TeleOp(name = "Driving TeleOp")
public class DriveTeleOp extends LinearOpMode {
    private MecanumDrive drivetrain;

    private Launcher launcher;
    private Slide slide;
    private Intake intake;
    private Outtake outtake;
    private DistanceSensor sensorDistance;
    private final double STOPPING_DISTANCE = 2; //inch
    private int SlideState;
    private boolean outtakeState;

    @Override
    public void runOpMode() {
        drivetrain = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        launcher = new Launcher(hardwareMap);
        slide = new Slide(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        sensorDistance = hardwareMap.get(DistanceSensor.class,"sensorDistance");
        SlideState = 0;
        outtakeState = false;

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

            if (gamepad2.square) SlideState = 1; //extend 50%
            else if (gamepad2.circle) SlideState = 2; //extend 100%
            else if (gamepad2.cross) SlideState = 0; //retracted
            slide.setState(SlideState);

            if (gamepad2.right_bumper) intake.setMotor(2);
            else if (gamepad2.left_bumper) intake.setMotor(1);
            else intake.setMotor(0);

            if (gamepad2.dpad_up) outtakeState = true; //dropping pixels
            if (gamepad2.dpad_down) outtakeState = false; //getting pixels
            outtake.setState(outtakeState);

            launcher.setState(gamepad2.touchpad);
            telemetry.update();
        }
    }
}
