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
@Autonomous(name = "redHome")
public class redHome extends LinearOpMode
{

    SampleMecanumDrive drive;
    private Pose2d posHigh = new Pose2d(-10,-23.5,0 );
    private Utilities utilities;
    private Object MacaroniHardware;
    private TrajectorySequence toHighJunction, adjust, parkLeft, parkRight, parkMid;

    OpenCvInternalCamera webcam;
    ColorDetection pipeline;
    String destination;



    @Override
    public void runOpMode() throws InterruptedException
    {
        drive= new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-45, -65, 90));
        buildTrajectories();

        MacaroniHardware hardware = new MacaroniHardware();
        utilities = new Utilities(hardware);
        hardware.init(hardwareMap);

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
                webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode)
            {

                // This will be called if the camera could not be opened

            }
        });

        utilities.openClaw();
        waitForStart();
        destination = pipeline.getParkPoint();

        if (!opModeIsActive())
        {
            return;
        }

        telemetry.addData("code received", "yay");
        telemetry.update();
        utilities.closeClaw();
        //utilities.wait(1000, telemetry);
        //utilities.moveArm( -2000);
        drive.followTrajectorySequence(toHighJunction);
        utilities.moveArm( -1275);
        utilities.wait(4500, telemetry);
        //drive.followTrajectorySequence(adjust);
        utilities.wait(1000, telemetry);
        utilities.openClaw();
        //utilities.wait(500,telemetry);
        //utilities.closeClaw();
        utilities.wait(1000,telemetry);
        utilities.moveArm(1275);
        utilities.wait(1000,telemetry);


        if (destination == "right")
        {
            drive.followTrajectorySequence(parkRight);

        }
        else if (destination == "left")
        {
            drive.followTrajectorySequence(parkLeft);

        }
        else
            drive.followTrajectorySequence(parkMid);

    }

    private void buildTrajectories()
    {
        toHighJunction = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .turn(Math.toRadians(-90))
                .forward(30)
                .waitSeconds(.10)
                .turn(Math.toRadians(90))
                .forward(24)
                .turn(Math.toRadians(-41))
                .forward(5.5)
                //.back(2.5)
                //.strafeLeft(14)
               // .forward(.5)
                .build();

        adjust = drive.trajectorySequenceBuilder(toHighJunction.end())
                .forward(7)
                .build();

    //teal
        parkRight = drive.trajectorySequenceBuilder(toHighJunction.end())
                .back(5)
                .turn(Math.toRadians(132))
                .forward(55)
                .strafeLeft(2)
                .build();

    //yellow
        parkLeft = drive.trajectorySequenceBuilder(toHighJunction.end())
                .back(7)
                .turn(Math.toRadians(-51))
                .forward(25)
                .build();

    //pink
        parkMid = drive.trajectorySequenceBuilder(toHighJunction.end())
                .back(7)
                .turn(Math.toRadians(39))
                .forward(15)
                .build();



    }
}