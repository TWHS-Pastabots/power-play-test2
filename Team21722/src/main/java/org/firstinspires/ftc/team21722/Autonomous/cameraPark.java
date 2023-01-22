package org.firstinspires.ftc.team21722.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;


import org.firstinspires.ftc.team21722.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team21722.Hardware.MacaroniHardware;
import org.firstinspires.ftc.team21722.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.team21722.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvInternalCamera;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Config
@Autonomous(name = "cameraPark")
public class cameraPark extends LinearOpMode
{

    SampleMecanumDrive drive;
    private Pose2d posHigh = new Pose2d(-10,-23.5,0 );
    private Utilities utilities;
    private TrajectorySequence moveRight;
    private TrajectorySequence moveLeft;
    private TrajectorySequence turnLeft;
    private TrajectorySequence inchForward;
    private Object MacaroniHardware;
    OpenCvInternalCamera webcam;
    ColorDetection pipeline;
    String destination;

    @Override
    public void runOpMode() throws InterruptedException
    {
        drive= new SampleMecanumDrive(hardwareMap);
        buildTrajectories();

        MacaroniHardware hardware = new MacaroniHardware();
        utilities = new Utilities(hardware);
        hardware.init(hardwareMap);

        utilities.openClaw();



        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new ColorDetection();
        webcam.setPipeline(pipeline);

        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()

        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 176, OpenCvCameraRotation.SIDEWAYS_LEFT);
                destination = pipeline.getParkPoint();
            }

            @Override
            public void onError(int errorCode)
            {

                // This will be called if the camera could not be opened

            }
        });

        waitForStart();
        destination = pipeline.getParkPoint();


        if (destination == "right")
        {
            utilities.closeClaw();
            telemetry.addData("yo", "the thing!");
            telemetry.update();
            drive.setMotorPowers(.25, .25, .25, .25);
            utilities.wait(2350);
            drive.setMotorPowers(0, 0, 0, 0);
            drive.followTrajectorySequence(moveLeft);
            drive.setMotorPowers(.4, .4, .4, .4);
            utilities.wait(2150);
            /*drive.followTrajectorySequence(turnLeft);
            utilities.wait(1000);
            drive.setMotorPowers(.2, .2, .2, .2);
            utilities.wait(1000);
            drive.setMotorPowers(0, 0, 0, 0);
            telemetry.addData("completed!", "the thing!");
            telemetry.update();*/

        }
        else if (destination == "left")
        {
            utilities.closeClaw();
            drive.setMotorPowers(.2, .2, .2, .2);
            utilities.wait(2150, telemetry);
            drive.setMotorPowers(0, 0, 0, 0);
            drive.followTrajectorySequence(moveRight);
            drive.setMotorPowers(.2, .2, .2, .2);
            utilities.wait(2150, telemetry);
            drive.setMotorPowers(0, 0, 0, 0);
        }
        else
            utilities.closeClaw();
            drive.setMotorPowers(.2, .2, .2, .2);
        utilities.wait(2500, telemetry);
        drive.setMotorPowers(0, 0, 0, 0);
    }

    public void buildTrajectories()
    {
        /*inchForward = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(8)
                .waitSeconds(2)
                .build();*/

        moveLeft = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .turn(Math.toRadians(95))
                //.forward(20)
                .build();

        moveRight = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .turn(Math.toRadians(-95))
                //.forward(12)
                .build();
        turnLeft = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .turn(Math.toRadians(-95))
                .build();
    }
}

