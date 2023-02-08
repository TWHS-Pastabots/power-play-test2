package org.firstinspires.ftc.team16909.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.team16909.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16909.hardware.FettucineHardware;
import org.firstinspires.ftc.team16909.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@Autonomous(name = "MJRight")

public class MJRight extends LinearOpMode {
    private SampleMecanumDrive drive;
    private Pose2d rightStart = new Pose2d(-36, 64, Math.toRadians(-90));
    String destination;

    int liftPos1 = 240;
    int armPos1 = 330;
    int armPosDrop = 240;

    private TrajectorySequence trajStart, trajMid, trajEndLeft, trajEndMiddle, trajEndRight;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;


    int LEFT = 0;
    int MIDDLE = 1;
    int RIGHT = 2;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {

        FettucineHardware hardware = new FettucineHardware();

        hardware.init(hardwareMap);

        hardware.leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        hardware.leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        hardware.rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        hardware.rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        hardware.armMotorOne.setDirection(DcMotorSimple.Direction.REVERSE);


        hardware.armMotorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        Utilities utilities = new Utilities(hardware);

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(rightStart);

        buildTrajectories();

        utilities.intake();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


        while (!isStarted()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == RIGHT || tag.id == MIDDLE) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight! :pog:\n\n Location data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }


        if (!opModeIsActive()) {
            return;
        }

        //
        // Start of autonomous
        //


        hardware.armMotorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //
        // Start of autonomous
        //          Demi was here
        //

        utilities.moveLift(liftPos1);
        utilities.moveArm(armPos1);

        utilities.wait(300);

        drive.followTrajectorySequence(trajStart);

        utilities.wait(500);
        utilities.moveArm(armPosDrop - armPos1);
        utilities.outtake();
        utilities.moveLift(-liftPos1);
        utilities.moveArm(armPos1 - armPosDrop);

        drive.followTrajectorySequence(trajMid);

        utilities.wait(500);
        utilities.moveArm(-armPos1);

        if (tagOfInterest.id == LEFT) {
            drive.followTrajectorySequence(trajEndLeft);
        } else if (tagOfInterest.id == MIDDLE) {
            drive.followTrajectorySequence(trajEndMiddle);
        } else {
            drive.followTrajectorySequence(trajEndRight);
        }

        utilities.moveArm(-armPos1);

    }

    public void buildTrajectories() {
        trajStart = drive.trajectorySequenceBuilder(rightStart)
                .forward(24)
                .turn(Math.toRadians(-48))
                .turn(Math.toRadians(90))
                .forward(5)
                .build();

        trajMid = drive.trajectorySequenceBuilder((trajStart.end()))
                .back(5)
                .turn(Math.toRadians(47))
                .build();

        trajEndRight = drive.trajectorySequenceBuilder(trajMid.end())
                .back(22)
                .turn(Math.toRadians(90))
                .build();

        //  trajEndMiddle (literally doesn't move :thumbsup: )


        trajEndLeft = drive.trajectorySequenceBuilder(trajMid.end())
                .forward(25)
                .build();
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}