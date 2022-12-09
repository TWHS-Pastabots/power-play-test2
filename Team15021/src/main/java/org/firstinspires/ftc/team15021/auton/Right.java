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
@Autonomous(name = "Right")
public class Right extends LinearOpMode
{
    SampleMecanumDrive drive;

    private Trajectory adjustment;
    private Trajectory adjustment2;
    private Trajectory park1;
    private Trajectory park2;
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
            drive.followTrajectory(adjustment2);
            utilities.moveArm01(455);
            utilities.wait(3000, telemetry);
            utilities.moveArm2(373);
            utilities.wait(5000, telemetry);
            utilities.tiltUp();
            drive.turn(Math.toRadians(-41));
            utilities.wait(500, telemetry);
            utilities.moveArm2(-50);
            utilities.wait(1000, telemetry);
            utilities.openClaw();
            utilities.wait(1000, telemetry);
            drive.turn(Math.toRadians(-139));
            utilities.moveArm2(-300);
            utilities.wait(1000, telemetry);
            utilities.moveArm01(-200);
            utilities.wait(3000, telemetry);
            drive.followTrajectory(park1);
        }
        if(zone==2)
        {
            drive.followTrajectory(adjustment);
            drive.turn(Math.toRadians(90));
            drive.followTrajectory(adjustment2);
            utilities.moveArm01(455);
            utilities.wait(3000, telemetry);
            utilities.moveArm2(373);
            utilities.wait(5000, telemetry);
            utilities.tiltUp();
            drive.turn(Math.toRadians(-41));
            utilities.wait(500, telemetry);
            utilities.moveArm2(-50);
            utilities.wait(1000, telemetry);
            utilities.openClaw();
            utilities.wait(1000, telemetry);
            drive.turn(Math.toRadians(-139));
            utilities.moveArm2(-300);
            utilities.wait(1000, telemetry);
            utilities.moveArm01(-200);
            utilities.wait(3000, telemetry);
            drive.followTrajectory(park2);
        }
        if(zone==3)
        {
            drive.followTrajectory(adjustment);
            drive.turn(Math.toRadians(90));
            drive.followTrajectory(adjustment2);
            utilities.moveArm01(455);
            utilities.wait(3000, telemetry);
            utilities.moveArm2(373);
            utilities.wait(5000, telemetry);
            utilities.tiltUp();
            drive.turn(Math.toRadians(-41));
            utilities.wait(500, telemetry);
            utilities.moveArm2(-50);
            utilities.wait(1000, telemetry);
            utilities.openClaw();
            utilities.wait(1000, telemetry);
            drive.turn(Math.toRadians(-139));
            utilities.moveArm2(-300);
            utilities.wait(1000, telemetry);
            utilities.moveArm01(-200);
            utilities.wait(3000, telemetry);
            drive.followTrajectory(park3);
        }





    }


    private void buildTrajectories()
    {


        adjustment = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(26.5).build();

        adjustment2 = drive.trajectoryBuilder(new Pose2d(26.5, 0, Math.toRadians(90)))
                .forward(25.75).build();

        park3= drive.trajectoryBuilder(new Pose2d(26.5,25.75, Math.toRadians(-90)))
                .forward(50.5).build();

        park2= drive.trajectoryBuilder(new Pose2d(26.5, 25.75, Math.toRadians(-90)))
                .forward(27.5).build();

        park1= drive.trajectoryBuilder(new Pose2d(26.5,25.75, Math.toRadians(-90)))
                .forward(5).build();


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