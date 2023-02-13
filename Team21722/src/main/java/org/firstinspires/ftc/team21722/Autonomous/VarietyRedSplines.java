package org.firstinspires.ftc.team21722.Autonomous;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team21722.Hardware.MacaroniHardware;
import org.firstinspires.ftc.team21722.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team21722.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


@Config
@Autonomous(name = "VarietyRedSplines")
public class VarietyRedSplines extends LinearOpMode
{

    SampleMecanumDrive drive;
    private Pose2d posHigh = new Pose2d(-10,-23.5,0 );
    private Utilities utilities;
    private Object MacaroniHardware;
    private TrajectorySequence toMidJunc, adjust, parkLeft, parkRight, parkMid , toConeStackM , toSmallJunc,toConeStackS,toHighJunc;

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

        utilities.closeClaw();
        drive.followTrajectorySequence(toMidJunc);
        utilities.moveArm( -900);
        utilities.wait(2000, telemetry);
        drive.followTrajectorySequence(adjust);
        utilities.openClaw();
        //6 seconds first score

        /*utilities.wait(500,telemetry);
        utilities.moveArm(-300); //measure height of the cone stack
        drive.followTrajectorySequence(toConeStackM);
        utilities.wait(1500,telemetry);//time seconds
        utilities.closeClaw();
        //obtained 1st cone at 9-10 seconds

        utilities.wait(500);
        utilities.moveArm(-500);
        drive.followTrajectorySequence(toSmallJunc);
        utilities.wait(1500);//time
        drive.followTrajectorySequence(adjust);
        utilities.openClaw();
        //Score 2nd cone at around 12-15 seconds

        utilities.wait(500,telemetry);
        utilities.moveArm(-300); //measure cone stack height
        drive.followTrajectorySequence(toConeStackS);
        utilities.wait(1500,telemetry);//time seconds
        utilities.closeClaw();
        //Obtained 2nd cone at 15-18 seconds

        utilities.wait(500);
        utilities.moveArm(-2200);
        drive.followTrajectorySequence(toHighJunc);
        utilities.wait(2000);//time seconds
        drive.followTrajectorySequence(adjust);
        utilities.openClaw();
        utilities.moveArm(2200);

        //Score 3rd cone at around 18-23 seconds
        //Parking should have around 5 seconds of time


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

         */

    }

    private void buildTrajectories()
    {
        toMidJunc = drive.trajectorySequenceBuilder(new Pose2d(-35, -65, 90))
                .splineToSplineHeading(new Pose2d(-30,-30,Math.toRadians(45)),Math.toRadians(30))
                .build();

        toConeStackM = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .back(5)
                .splineToSplineHeading(new Pose2d(-64,-12,Math.toRadians(180)),Math.toRadians(45))
                .build();

        toSmallJunc = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .splineToSplineHeading(new Pose2d(-48,-15,Math.toRadians(-90)),Math.toRadians(180))
                .build();

        toConeStackS = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .splineToSplineHeading(new Pose2d(-60,-12,Math.toRadians(180)),Math.toRadians(-90))
                .build();

        toHighJunc = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .splineToSplineHeading(new Pose2d(-25,-8,Math.toRadians(90)),Math.toRadians(180))
                .build();

        adjust = drive.trajectorySequenceBuilder(toMidJunc.end())
                .forward(5)
                .build();

        //teal
        parkRight = drive.trajectorySequenceBuilder(adjust.end())
                .back(5)
                .strafeRight(5)
                .forward(28)
                .build();

        //yellow
        parkLeft = drive.trajectorySequenceBuilder(adjust.end())
                .back(5)
                .turn(Math.toRadians(90))
                .forward(10)
                .build();

        //pink
        parkMid = drive.trajectorySequenceBuilder(adjust.end())
                .back(5)
                .strafeLeft(5)
                .build();



    }
}



