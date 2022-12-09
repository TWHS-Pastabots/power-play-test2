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

@Autonomous(name = "HighJunctionLeft")

public class HighJunctionLeft extends LinearOpMode
{
    private SampleMecanumDrive drive;
    private Pose2d rightStart = new Pose2d(-36, 64, Math.toRadians(-90));
    String destination;

    int liftPos1 = 2000; //3897
    int armPos1 = 330;
    int armPosDrop = 240;

    private TrajectorySequence trajStart, trajEndLeft, trajEndMiddle, trajEndRight;

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

                // This will be called if the camera could not be opened

            }
        });


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



        if (!opModeIsActive())
        {
            return;
        }

        //
        // Start of autonomous
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

        if (destination == "left")
        {
            drive.followTrajectorySequence(trajEndLeft);
        }
        else if (destination == "middle")
        {
            drive.followTrajectorySequence(trajEndMiddle);
        }
        else
        {
            drive.followTrajectorySequence(trajEndRight);
        }

        utilities.moveArm(-armPos1);
    }

    public void buildTrajectories()
    {
        trajStart = drive.trajectorySequenceBuilder(rightStart)
                .forward(25)
                .turn(Math.toRadians(-86))
                .forward(23)
                .turn(Math.toRadians(47))
                .forward(4)
                .build();

        trajEndRight = drive.trajectorySequenceBuilder(trajStart.end())
                .back(2)
                .waitSeconds(1)
                .turn(Math.toRadians(-128))
                .build();

        trajEndMiddle = drive.trajectorySequenceBuilder(trajStart.end())
                .back(2)
                .turn(Math.toRadians(-47))
                .back(24)
                .build();

        trajEndLeft = drive.trajectorySequenceBuilder(trajStart.end())
                .back(2)
                .turn(Math.toRadians((-47)))
                .back(48)
                .build();
    }
}