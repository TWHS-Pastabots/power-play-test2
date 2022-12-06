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

@Autonomous(name = "HighJunction")

public class HighJunction extends LinearOpMode
{
    private SampleMecanumDrive drive;
    private Pose2d rightStart = new Pose2d(-36, 64, Math.toRadians(-90));

    int liftPos1 = 3500; //3897
    int armPos1 = 250;

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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
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
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });


        Utilities utilities = new Utilities(hardware);

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(rightStart);

        buildTrajectories();

        utilities.intake();

        /*ColorDetectionPipeline pipeline = new ColorDetectionPipeline();

        if (pipeline.getChosenColor().equals(ColorDetectionPipeline.color.LAVENDER))
        {
            finalTraj = endTraj1;
        }

        if (pipeline.getChosenColor().equals(ColorDetectionPipeline.color.DARKGREEN))
        {
            finalTraj = endTraj2;
        }
        if (pipeline.getChosenColor().equals(ColorDetectionPipeline.color.LIGHTBLUE))
        {
            finalTraj = endTraj3;
        }
        */

        waitForStart();

        if (!opModeIsActive())
        {
            return;
        }

        utilities.moveArm(armPos1);
        utilities.wait(500);

        utilities.moveLift(liftPos1);
        utilities.wait(500);

        drive.followTrajectorySequence(traj1);

        utilities.wait(500);

        //utilities.wait(500, telemetry);

        utilities.outtake();
        utilities.wait(700);
        utilities.moveArm(0);
    }

    public void buildTrajectories()
    {
        traj1 = drive.trajectorySequenceBuilder(rightStart)
                .forward(25)
                .waitSeconds(.05)
                .turn(Math.toRadians(90))
                .waitSeconds(.05)
                .forward(24)
                .waitSeconds(.05)
                .turn(Math.toRadians(-45))
                .waitSeconds(.05)
                .forward(4)
                .waitSeconds(1)
                .forward(-4)
                .turn(Math.toRadians(-45))
                .build();
    }
}
