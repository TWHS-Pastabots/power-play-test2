package org.firstinspires.ftc.team15021.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team15021.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team15021.hardware.RavioliHardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Config
@Autonomous(name = "Alt")
public class Alt extends LinearOpMode
{
    SampleMecanumDrive drive;

    private Trajectory adjustment;
    private Trajectory park1;
    private Trajectory park3;

    OpenCvInternalCamera webcam;
    SignalPipeline pipeline;


    public void runOpMode() throws InterruptedException
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new SignalPipeline();
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
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        RavioliHardware hardware = new RavioliHardware();
        util utilities = new util(hardware);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 0, 0));
        hardware.init(hardwareMap);

        while (!gamepad1.triangle)
        {
            telemetry.addData("H:", pipeline.avg);
            telemetry.update();
        }

        buildTrajectories();
        utilities.closeClaw();
        utilities.resetClaw();

        waitForStart();
        if(!opModeIsActive()) {return;}

        SignalPipeline.ans zone0 = pipeline.getAnalysis();
        int zone = 3;
        zone = signalToInt(zone0);

        if(zone==1)
        {
            drive.followTrajectory(adjustment);
            drive.turn(Math.toRadians(90));
            drive.followTrajectory(park1);
        }
        if(zone==2)
        {
            drive.followTrajectory(adjustment);
        }
        if(zone==3)
        {
            drive.followTrajectory(adjustment);
            drive.turn(Math.toRadians(-90));
            drive.followTrajectory(park3);

        }





    }


    private void buildTrajectories()
    {


        adjustment = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(48.5).build();

        park1= drive.trajectoryBuilder(new Pose2d(50,0, Math.toRadians(90)))
                .forward(24).build();

        park3= drive.trajectoryBuilder(new Pose2d(50,0, Math.toRadians(-90)))
                .forward(24).build();


    }

    private int signalToInt(SignalPipeline.ans zone)
    {

        switch (zone)
        {
            case GREEN:
                return 2;

            case PURPLE:
                return 1;

            case ORANGE:
                return 3;
        }

        return 0;
    }
}
