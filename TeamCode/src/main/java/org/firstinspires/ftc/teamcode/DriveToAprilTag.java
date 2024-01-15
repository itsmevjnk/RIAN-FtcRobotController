package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Drive to AprilTag")
public class DriveToAprilTag extends LinearOpMode {
    private MecanumDrivetrain drivetrain;

    /* camera/CV */
    private VisionPortal visionPortal;
    private boolean cameraStreamingPaused = false;
    private AprilTagProcessor aprilTag; // AprilTag detector

    private static final int DESIRED_TAG_ID = 1;
    private static final double DESIRED_DISTANCE = 10; // inch

    private static final double DRIVE_GAIN = 0.02;
    private static final double STRAFE_GAIN = 0.005;
    private static final double TURN_GAIN = 0.01;
    private static final boolean INVERT_AUTO_CONTROLS = true; // set to true if camera is mounted at the back

    private static final double MAX_AUTO_DRIVE = 0.75;
    private static final double MAX_AUTO_STRAFE = 0.75;
    private static final double MAX_AUTO_TURN = 0.25;

    private static final double DISTANCE_ERR_TOLERANCE = 0.15;
    private static final double HEADING_ERR_TOLERANCE = 1.0;
    private static final double YAW_ERR_TOLERANCE = 1.0;

    @Override
    public void runOpMode() {
        drivetrain = new MecanumDrivetrain(hardwareMap, telemetry);

        /* initialize AprilTag */
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(1); // TODO: change this to adapt to gameplay conditions
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, RobotConfig.WEBCAM_BACK))
                .addProcessor(aprilTag)
                .build();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        setManualExposure(6, 150);

        waitForStart();

        boolean unattendedDrive = false;
        boolean rbPressed = false;
        while(opModeIsActive()) {
            /* detect AprilTags */
            AprilTagDetection detectedTag = null;
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for(AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null && detection.id == DESIRED_TAG_ID) { // found our tag
                    detectedTag = detection;
                    break;
                }
            }

            if(detectedTag != null) {
                telemetry.addData(">", "Hold left bumper to drive towards detected tag (right bumper for unattended driving):");
                telemetry.addData("Tag", "ID %d (%s)", detectedTag.id, detectedTag.metadata.name);
                telemetry.addData("Range", "%5.1f in.", detectedTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f deg.", detectedTag.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f deg.", detectedTag.ftcPose.yaw);
            } else telemetry.addData(">", "Drive robot around to pick up a tag");

            if(gamepad1.right_bumper) {
                if(!rbPressed) {
                    rbPressed = true;
                    unattendedDrive = !unattendedDrive;
                }
            } else rbPressed = false;
            telemetry.addData("Unattended drive", unattendedDrive);

            if(detectedTag != null && (gamepad1.left_bumper || unattendedDrive)) {
                /* drive towards detected tag - TODO: figure out how to strafe */
                double rangeError = detectedTag.ftcPose.range - DESIRED_DISTANCE;
                double headingError = -detectedTag.ftcPose.bearing; // positive bearing = turn counter-clockwise = negative turn value
                double yawError = detectedTag.ftcPose.yaw;
                if(INVERT_AUTO_CONTROLS) { // reverse X/Y corrections but not rotation
                    rangeError *= -1.0;
//                    headingError *= -1.0;
                    yawError *= -1.0;
                }

                if(Math.abs(rangeError) <= DISTANCE_ERR_TOLERANCE && Math.abs(headingError) <= HEADING_ERR_TOLERANCE && Math.abs(yawError) <= YAW_ERR_TOLERANCE)
                    unattendedDrive = false; // no more driving if we've reached our target within tolerances

                if(gamepad1.left_bumper || unattendedDrive) {
                    double x = Range.clip(yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                    double y = Range.clip(rangeError * DRIVE_GAIN, -MAX_AUTO_DRIVE, MAX_AUTO_DRIVE);
                    double rot = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                    drivetrain.drive(x, y, rot);
                }
            } else {
                /* normal driving */
                drivetrain.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x); // NOTE: top joystick positions result in negative Y reading so we need to negate it
            }

            telemetry.update();
        }
    }

    private void setManualExposure(int msec, int gain) {
        if(visionPortal == null) return;

        if(visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while(!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING))
                sleep(20); // wait until camera goes live
        }

        if(!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if(exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual); // set to manual exposure
                sleep(50);
            }
            exposureControl.setExposure((long)msec, TimeUnit.MILLISECONDS);
            sleep(20);

            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}
