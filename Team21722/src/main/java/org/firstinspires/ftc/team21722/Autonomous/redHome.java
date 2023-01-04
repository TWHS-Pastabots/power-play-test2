package org.firstinspires.ftc.team21722.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;


import org.firstinspires.ftc.team21722.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team21722.Hardware.MacaroniHardware;
import org.firstinspires.ftc.team21722.trajectorysequence.TrajectorySequence;


@Config
@Autonomous(name = "redHome")
public class redHome extends LinearOpMode
{

    SampleMecanumDrive drive;
    private Pose2d posHigh = new Pose2d(-10,-23.5,0 );
    private Utilities utilities;
    private Object MacaroniHardware;
    private TrajectorySequence toHighJunction;

    @Override
    public void runOpMode() throws InterruptedException
    {
        drive= new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-45, -65, 90));
        buildTrajectories();

        MacaroniHardware hardware = new MacaroniHardware();
        utilities = new Utilities(hardware);
        hardware.init(hardwareMap);

        utilities.openClaw();
        waitForStart();

        if (!opModeIsActive())
        {
            return;
        }


        utilities.closeClaw();
        drive.followTrajectorySequence(toHighJunction);
        utilities.moveArm( -2000);
        utilities.wait(6500, telemetry);
        utilities.openClaw();
        utilities.moveArm(1400);
        utilities.wait(6500,telemetry);
    }

    private void buildTrajectories()
    {
        toHighJunction = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .turn(Math.toRadians(-90))
                .forward(30)
                .waitSeconds(.10)
                .turn(Math.toRadians(90))
                .forward(29)
                .turn(Math.toRadians(-90))
                .back(1.5)
                .strafeLeft(12)
                .forward(.5)
                .build();


    }
}
