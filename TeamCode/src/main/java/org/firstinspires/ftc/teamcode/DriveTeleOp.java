package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Launcher;

@TeleOp(name = "Driving TeleOp")
public class DriveTeleOp extends LinearOpMode {
    private MecanumDrivetrain drivetrain;
    private Launcher launcher;

    @Override
    public void runOpMode() {
        drivetrain = new MecanumDrivetrain(hardwareMap, telemetry);
        launcher = new Launcher(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            drivetrain.updatePosition();
            drivetrain.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x); // NOTE: top joystick positions result in negative Y reading so we need to negate it
            launcher.setState(gamepad1.options);
            telemetry.update();
        }
    }
}
