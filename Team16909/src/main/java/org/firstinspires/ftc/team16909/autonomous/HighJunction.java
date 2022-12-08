package org.firstinspires.ftc.team16909.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.team16909.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16909.hardware.FettucineHardware;
import org.firstinspires.ftc.team16909.trajectorysequence.TrajectorySequence;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "HighJunction")

public class HighJunction extends LinearOpMode
{
    private SampleMecanumDrive drive;
    private Pose2d rightStart = new Pose2d(-36, 64, Math.toRadians(-90));

    int liftPos1 = 2000; //3897
    int armPos1 = 300;

    private TrajectorySequence traj1, finalTraj, endTraj1, endTraj2, endTraj3;

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


        /*int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new ColorDetectionPipeline();
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
        });*/




        Utilities utilities = new Utilities(hardware);

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(rightStart);

        /*while (!gamepad1.triangle)
        {
            telemetry.addData("Hue Value: ", pipeline.meanCol);
            telemetry.addData("Chosen Color: ", pipeline.getColor());
            telemetry.addData("Park Point: ", pipeline.getParkPoint());
            telemetry.update();
            utilities.wait(100);
            telemetry.clear();
        }*/

        buildTrajectories();

        utilities.intake();


        waitForStart();

        if (!opModeIsActive())
        {
            return;
        }

        //utilities.wait(500);

        utilities.moveLift(liftPos1);
        utilities.moveArm(armPos1);


        drive.followTrajectorySequence(traj1);

        utilities.outtake();
        utilities.moveLift(-liftPos1);

        drive.followTrajectorySequence(endTraj1);

        utilities.moveArm(-armPos1);
    }

    public void buildTrajectories()
    {
        traj1 = drive.trajectorySequenceBuilder(rightStart)
                .forward(27)
                //.waitSeconds(.05)
                .turn(Math.toRadians(95))
                //.waitSeconds(.05)
                .forward(25)
                //.waitSeconds(.05)
                .turn(Math.toRadians(-45))
                //.waitSeconds(.05)
                //.forward(3)
                //.waitSeconds(1)
                .build();

        endTraj1 = drive.trajectorySequenceBuilder(traj1.end())
                //.forward(-4)
                .turn(Math.toRadians(45))
                .forward(-24)
                .build();
    }
}
