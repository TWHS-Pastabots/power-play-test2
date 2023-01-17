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
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "HJFarLeft")

public class HJFarLeft extends LinearOpMode
{
    private SampleMecanumDrive drive;
    private Pose2d rightStart = new Pose2d(-36, 64, Math.toRadians(-90));
    String destination;

    int liftPos1 = 378;
    int armPos1 = 330;
    int armPosDrop = 270;

    private TrajectorySequence trajStart, trajMid, trajEndLeft, trajEndMiddle, trajEndRight;

    OpenCvInternalCamera webcam;
    ColorDetectionPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException
    {

        FettucineHardware hardware = new FettucineHardware();

        hardware.init(hardwareMap);

        hardware.leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        hardware.leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        hardware.rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        hardware.rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        hardware.armMotorOne.setDirection(DcMotorSimple.Direction.REVERSE);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new ColorDetectionPipeline();
        pipeline.isRight = false;
        webcam.setPipeline(pipeline);

        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode)
            {

                // This will be called if the camera could not be opened

            }
        });

        hardware.armMotorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


//cyan 103, yellow 37, magenta 160

        Utilities utilities = new Utilities(hardware);

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(rightStart);

        while (!gamepad1.triangle)
        {
            telemetry.addData("Hue Value: ", pipeline.meanCol);
            telemetry.addData("Chosen Color: ", pipeline.getColor());
            telemetry.addData("Park Point: ", pipeline.getParkPoint());
            telemetry.update();
            utilities.wait(100);
            telemetry.clear();
        }

        buildTrajectories();

        utilities.intake();

        waitForStart();

        hardware.armMotorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        if (!opModeIsActive())
        {
            return;
        }

        //
        // Start of autonomous
        //          Demi was here
        //

        utilities.moveLift(liftPos1);
        utilities.moveArm(armPos1);

        utilities.wait(300);
        destination = pipeline.getParkPoint();

        drive.followTrajectorySequence(trajStart);

        utilities.wait(500);
        utilities.moveArm(armPosDrop-armPos1);
        utilities.outtake();
        utilities.moveLift(-liftPos1);
        utilities.moveArm(armPos1-armPosDrop);

        drive.followTrajectorySequence(trajMid);

        utilities.wait(500);
        utilities.moveArm(-armPos1);

        if (destination.equals("left"))
        {
            drive.followTrajectorySequence(trajEndLeft);
        }
        else if(destination.equals("right"))
        {
            drive.followTrajectorySequence(trajEndRight);
        }

    }

    public void buildTrajectories()
    {
        trajStart = drive.trajectorySequenceBuilder(rightStart)
                .forward(48)
                .turn(Math.toRadians(48))
                .turn(Math.toRadians(-90))
                .forward(5)
                .build();

        trajMid = drive.trajectorySequenceBuilder((trajStart.end()))
                .back(5)
                .turn(Math.toRadians(43))
                .back(22)
                .turn(Math.toRadians(268))
                .build();

        trajEndRight = drive.trajectorySequenceBuilder(trajMid.end())
                .forward(22)
                .turn(Math.toRadians(-90))
                .build();

        //  trajEndMiddle (doesn't move :thumbsup: )

        trajEndLeft = drive.trajectorySequenceBuilder(trajMid.end())
                .back(25)
                .build();
    }
}