package org.firstinspires.ftc.team15021.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team15021.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team15021.hardware.RavioliHardware;

@Config
@Autonomous(name = "Right")
public class Right extends LinearOpMode
{
    SampleMecanumDrive drive;

    private Pose2d interim = new Pose2d(11,0,0);
    private Pose2d interim2 = new Pose2d(17,-23,Math.toRadians(-90));
    private Pose2d interim3 = new Pose2d(23.5, -23, Math.toRadians(-90));

    private Trajectory adjustment;
    private Trajectory adjustment2;
    private Trajectory adjustment3;


    public void runOpMode()
    {
        RavioliHardware hardware = new RavioliHardware();
        util utilities = new util(hardware);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 0, 0));
        hardware.init(hardwareMap);

        buildTrajectories();
        utilities.closeClaw();
        utilities.resetClaw();

        waitForStart();
        if(!opModeIsActive()) {return;}

        drive.turn(Math.toRadians(90));
        drive.setPoseEstimate(new Pose2d(0, 0, 0));
        drive.followTrajectory(adjustment);
        drive.followTrajectory(adjustment2);
        drive.followTrajectory(adjustment3);
        utilities.moveArm01(450);
        utilities.wait(3000, telemetry);
        utilities.moveArm2(350);
        utilities.wait(5000, telemetry);
        utilities.tiltUp();
        drive.turn(Math.toRadians(76));
        utilities.moveArm2(-20);
        utilities.wait(1000, telemetry);
        utilities.openClaw();
        utilities.wait(1000, telemetry);
        drive.turn(Math.toRadians(-76));
        utilities.moveArm2(-300);
        utilities.wait(1000, telemetry);
        utilities.moveArm01(-200);
        utilities.wait(3000, telemetry);




    }


    private void buildTrajectories()
    {

        /*adjustment = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .strafeLeft(29).build();

        adjustment2 = drive.trajectoryBuilder(new Pose2d(0, 29, 0))
                .forward(31).build();*/

        adjustment = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToLinearHeading(interim, 0)
                .build();
        adjustment2 = drive.trajectoryBuilder(adjustment.end())
                .splineToLinearHeading(interim2, Math.toRadians(-90))
                .build();
        adjustment3 = drive.trajectoryBuilder(adjustment2.end())
                .splineToLinearHeading(interim3, Math.toRadians(-45))
                .build();
    }

}
