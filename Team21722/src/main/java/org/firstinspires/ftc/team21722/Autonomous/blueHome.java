package org.firstinspires.ftc.team21722.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;


import org.firstinspires.ftc.team21722.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team21722.Hardware.MacaroniHardware;
import org.firstinspires.ftc.team21722.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvInternalCamera;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


@Config
@Autonomous(name = "blueHome")
public class blueHome extends LinearOpMode
{

    SampleMecanumDrive drive;
    private Pose2d posHigh = new Pose2d(-10,-23,0 );
    private Utilities utilities;
    private Object MacaroniHardware;
    private TrajectorySequence toHighJunction, parkLeft, parkMiddle, parkRight;
    String destination;

    OpenCvInternalCamera webcam;
    ColorDetection pipeline;



    @Override
    public void runOpMode() throws InterruptedException
    {
        drive= new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-45, 65, 90));
        buildTrajectories();

        MacaroniHardware hardware = new MacaroniHardware();
        utilities = new Utilities(hardware);
        hardware.init(hardwareMap);

        utilities.openClaw();
        //waitForStart();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new ColorDetection();
        webcam.setPipeline(pipeline);
        destination = pipeline.getParkPoint();

        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()

        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 176, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode)
            {

                // This will be called if the camera could not be opened

            }
        });



        utilities.wait(500,telemetry);
        destination = pipeline.getParkPoint();

        Utilities utilities = new Utilities(hardware);

        drive = new SampleMecanumDrive(hardwareMap);
        buildTrajectories();

        waitForStart();

        if(!opModeIsActive())
        {
            return;
        }



        telemetry.addData("Hue Value: ", pipeline.meanCol);
        telemetry.addData("Chosen Color: ", pipeline.getColor());
        telemetry.addData("Park Point: ", pipeline.getParkPoint());
        telemetry.update();


        utilities.closeClaw();
        drive.followTrajectorySequence(toHighJunction);
        utilities.moveArm( -1900);
        utilities.wait(6500, telemetry);
        utilities.openClaw();
        utilities.moveArm(1900);
        utilities.wait(6500,telemetry);


        if(destination == "left")
        {
            drive.followTrajectorySequence(parkLeft);
        }
        else if(destination == "middle")
        {
            drive.followTrajectorySequence(parkMiddle);
        }
        else
        {
            drive.followTrajectorySequence(parkRight);
        }
    }

    private void buildTrajectories()
    {
        toHighJunction = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(1)
                .turn(Math.toRadians(90))
                .forward(11)
                .waitSeconds(.10)
                .turn(Math.toRadians(-90))
                .forward(25)
                .turn(Math.toRadians(90))
                .back(3)
                .forward(1)
                .strafeRight(9.5)
                .forward(1.5)
                .build();




        parkRight = drive.trajectorySequenceBuilder(toHighJunction.end())
                .strafeLeft(10)
                .back(20)
                .build();
        parkLeft = drive.trajectorySequenceBuilder(toHighJunction.end())
                .strafeLeft(10)
                .forward(17)
                .build();
        parkMiddle = drive.trajectorySequenceBuilder(toHighJunction.end())
                .strafeLeft(10)
                .build();

    }
}
