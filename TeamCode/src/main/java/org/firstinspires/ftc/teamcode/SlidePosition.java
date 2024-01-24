package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Slide Position Calibration")
public class SlidePosition extends LinearOpMode {
    private DcMotor slideLeft;
    private DcMotor slideRight;

    @Override
    public void runOpMode() {
        slideLeft = hardwareMap.get(DcMotor.class, RobotConfig.DC_SLIDE_LEFT);
        slideRight = hardwareMap.get(DcMotor.class, RobotConfig.DC_SLIDE_RIGHT);

        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            slideLeft.setPower(-gamepad1.left_stick_y);
            slideRight.setPower(-gamepad1.right_stick_y);
            telemetry.addData("Left slide pos.", slideLeft.getCurrentPosition());
            telemetry.addData("Right slide pos.", slideRight.getCurrentPosition());
            telemetry.update();
        }
    }
}
