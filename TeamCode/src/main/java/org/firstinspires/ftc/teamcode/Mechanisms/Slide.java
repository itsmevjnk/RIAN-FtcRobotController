package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConfig;

public class Slide {
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private final int LEFT_POSITION = -6000;
    private final int RIGHT_POSITION = 0;
    private final double SPEED = 0.75;
    public Slide (HardwareMap hardwareMap) {
        motorLeft = hardwareMap.get(DcMotor.class, RobotConfig.DC_SLIDE_LEFT);
        motorRight = hardwareMap.get(DcMotor.class, RobotConfig.DC_SLIDE_RIGHT);
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setPower(-SPEED);
        motorRight.setPower(-SPEED);
        motorLeft.setTargetPosition(0);
        motorRight.setTargetPosition(0);
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setState(boolean state) {
        if (state) {
            motorLeft.setTargetPosition(LEFT_POSITION);
            motorRight.setTargetPosition(RIGHT_POSITION);
        } else {
            motorLeft.setTargetPosition(0);
            motorRight.setTargetPosition(0);
        }
    }
}
