package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConfig;

public class Intake {
    private DcMotor motor;
    private final double SPEED = 1.0;
    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, RobotConfig.DC_INTAKE);
        motor.setPower(0);
    }
    public void setMotor(boolean state) {
        motor.setPower((state) ? SPEED : 0);
    }
}
