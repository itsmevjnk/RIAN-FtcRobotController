package org.firstinspires.ftc.teamcode.Mechanisms;

import org.firstinspires.ftc.teamcode.RobotConfig;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

public class Launcher {
    private CRServo servo;

    private final double SPEED = 1.0;

    public Launcher(HardwareMap hardwareMap) {
        servo = hardwareMap.get(CRServo.class, RobotConfig.SERVO_LAUNCHER);
        servo.setPower(0);
    }

    public void setState(boolean state) {
        servo.setPower((state) ? SPEED : 0);
    }
}
