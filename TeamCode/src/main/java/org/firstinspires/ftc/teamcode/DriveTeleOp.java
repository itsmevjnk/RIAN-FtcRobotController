package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.Mechanisms.Slide;

@TeleOp(name = "Driving TeleOp")
public class DriveTeleOp extends LinearOpMode {
    private MecanumDrive drivetrain;

    private Launcher launcher;
    private Slide slide;

    @Override
    public void runOpMode() {
        drivetrain = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        launcher = new Launcher(hardwareMap);
        slide = new Slide(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            drivetrain.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));
            drivetrain.updatePoseEstimate();

            telemetry.addData("x", drivetrain.pose.position.x);
            telemetry.addData("y", drivetrain.pose.position.y);
            telemetry.addData("Heading", Math.toDegrees(drivetrain.pose.heading.toDouble()));

            launcher.setState(gamepad1.options);
            slide.setState(gamepad1.cross);
            telemetry.update();
        }
    }
}
