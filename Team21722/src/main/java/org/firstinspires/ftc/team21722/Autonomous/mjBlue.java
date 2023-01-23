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
@Autonomous(name = "mjBlue")
public class mjBlue extends LinearOpMode
{

    SampleMecanumDrive drive;
    private Pose2d posHigh = new Pose2d(-10,-23,0 );
    private Utilities utilities;
    private Object MacaroniHardware;
    private TrajectorySequence toMidJunc, parkLeft, parkMiddle, parkRight, adjust;
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

        if(!opModeIsActive())
        {
            return;
        }




        utilities.closeClaw();
        drive.followTrajectorySequence(toMidJunc);
        utilities.moveArm( -1300);
        utilities.wait(3500, telemetry);
        drive.followTrajectorySequence(adjust);
        utilities.openClaw();
        utilities.wait(500,telemetry);
        utilities.closeClaw();
        utilities.moveArm(1300);
        utilities.wait(3500,telemetry);


        if (destination == "right")
        {
            drive.followTrajectorySequence(parkRight);

        }
        else if (destination == "left")
        {
            drive.followTrajectorySequence(parkLeft);

        }
        else
            drive.followTrajectorySequence(parkMiddle);
    }

    private void buildTrajectories()
    {
        toMidJunc = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(27)
                .turn(Math.toRadians(45))
                .build();


        adjust = drive.trajectorySequenceBuilder(toMidJunc.end())
                .forward(5)
                .build();



        //teal
        parkRight = drive.trajectorySequenceBuilder(adjust.end())
                .back(5)
                .turn(Math.toRadians(45))
                .forward(23)
                .build();

        //yellow
        parkLeft = drive.trajectorySequenceBuilder(adjust.end())
                .back(5)
                .turn(Math.toRadians(-135))
                .forward(25)
                .build();


        //pink
        parkMiddle = drive.trajectorySequenceBuilder(adjust.end())
                .back(5)
                .turn(Math.toRadians(-45))
                .forward(10)
                .build();

    }
}
