package org.firstinspires.ftc.team16909.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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

@Autonomous(name = "HJLeftTwoCone")

public class HJLeftTwoCone extends LinearOpMode
{
    private SampleMecanumDrive drive;
    private Pose2d rightStart = new Pose2d(-36, 64, Math.toRadians(-90));
    String destination;

    int liftPos1 = 262;
    int liftPos2 = 358;
    int liftPosOuttake = 60;
    int armPos1 = 465;
    int armPosGrab = -385;

    /*

     */
    private TrajectorySequence traj1, traj2, traj3, traj4, traj2A, trajEndRight, trajEndLeft;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    //      dw we did those :)
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
                    telemetry.addLine("Tag of interest is in sight! :pog:\n\nLocation data:");
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


        hardware.armMotorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //
        // Start of autonomous
        //          Demi was here
        //

        /// prep for mid junction
        utilities.moveLift(liftPos1);
        utilities.moveArm(armPos1);
        drive.followTrajectorySequence(traj1); // goes to mid junction

        /// at mid junction
        utilities.moveLift(-liftPosOuttake);
        utilities.wait(250);
        utilities.outtake();
        utilities.wait(200);
        utilities.intake();
        utilities.moveArm(armPosGrab);
        utilities.moveLift(-liftPos1 + liftPosOuttake);

        /// leaves mid junction, goes to stack and gets cone
        drive.followTrajectorySequence(traj2); //   goes to stack
        utilities.outtake();
        drive.followTrajectorySequence(traj2A);
        utilities.wait(750);
        utilities.intake();

        /// prep for high junction
        utilities.moveLift(liftPos2);
        utilities.wait(500);
        utilities.moveArm(-armPosGrab - 30);
        drive.followTrajectorySequence(traj3); // goes to high junction

        /// at high junction
        utilities.moveLift(-liftPosOuttake);
        utilities.wait(250);
        utilities.outtake();
        utilities.wait(250);
        utilities.intake();
        utilities.moveArm(armPosGrab);
        utilities.wait(750);
        utilities.moveLift(-liftPos2 + liftPosOuttake);
        utilities.wait(100);

        /// leaves high junction and parks
        drive.followTrajectorySequence(traj4);
        utilities.wait(500);

        if (tagOfInterest!= null &&  tagOfInterest.id == LEFT)
        {
            drive.followTrajectorySequence(trajEndLeft);
        }
        else if (tagOfInterest!= null && tagOfInterest.id == RIGHT)
        {
            drive.followTrajectorySequence(trajEndRight);
        }

        utilities.moveArm(-20);
    }

    public void buildTrajectories()
    {
        traj1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(50)
                .turn(Math.toRadians(43))
                .back(10.5)
                .build();

        // medium junction to stack
        traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .forward(9)
                .turn(Math.toRadians(47))
                .forward(11)
                .build();

        traj2A = drive.trajectorySequenceBuilder(traj2.end())
                .forward(11)
                .build();
        // stack to high junction
        traj3 = drive.trajectorySequenceBuilder(traj2A.end())
                .back(20)
                .turn(Math.toRadians(45))
                .back(11)
                .build();
        // high junction to parking
        traj4 = drive.trajectorySequenceBuilder(traj3.end())
                .forward(11)
                .turn(Math.toRadians(-45))
                .build();

        trajEndLeft = drive.trajectorySequenceBuilder(traj4.end())
                .forward(23)
                .build();

        //  trajEndMiddle (literally doesn't move :thumbsup: )
        trajEndRight = drive.trajectorySequenceBuilder(traj4.end())
                .back(26)
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