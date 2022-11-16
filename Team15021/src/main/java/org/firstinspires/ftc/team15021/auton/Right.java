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
    private Pose2d interim = new Pose2d(0, 8, 0);
    private Pose2d pos1 = new Pose2d(0, 17, 0);
    private Pose2d posHigh = new Pose2d(33, 29.25, 0); //change y to 33.5?

    //private Trajectory adjustment;
    private Trajectory toHighJunction;


    public void runOpMode()
    {
        RavioliHardware hardware = new RavioliHardware();
        util utilities = new util(hardware);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(2.5, 0, 0));
        hardware.init(hardwareMap);

        buildTrajectories();
        utilities.closeClaw();


        waitForStart();
        if(!opModeIsActive()) {return;}

        drive.followTrajectory(toHighJunction);
        utilities.moveArm01(450);
        utilities.wait(3000);
        utilities.moveArm2(375);
        utilities.wait(5000);
        utilities.tiltDown();
        drive.turn(Math.toRadians(85));
        utilities.wait(1000);
        utilities.openClaw();
        utilities.wait(1000);




    }


    private void buildTrajectories()
    {

        toHighJunction = drive.trajectoryBuilder(drive.getPoseEstimate(), Math.toRadians(90))
                .splineToSplineHeading(interim, Math.toRadians(90))
                .splineToSplineHeading(pos1, Math.toRadians(90))
                .splineToLinearHeading(posHigh, 0).build();


    }

}
