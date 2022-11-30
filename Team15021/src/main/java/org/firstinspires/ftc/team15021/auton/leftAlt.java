package org.firstinspires.ftc.team15021.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team15021.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team15021.hardware.RavioliHardware;

@Config
@Autonomous(name = "LeftAlt")
public class leftAlt extends LinearOpMode
{
    SampleMecanumDrive drive;
    private Trajectory adjustment;
    private Trajectory adjustment2;


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

        drive.followTrajectory(adjustment);
        utilities.moveArm01(450);
        utilities.wait(3000, telemetry);
        utilities.moveArm2(373);
        utilities.wait(5000, telemetry);
        utilities.tiltUp();
        drive.turn(Math.toRadians(-43));
        utilities.moveArm2(-20);
        utilities.wait(1000, telemetry);
        utilities.openClaw();
        utilities.wait(1000, telemetry);
        drive.turn(Math.toRadians(43));
        utilities.moveArm2(-300);
        utilities.wait(1000, telemetry);
        utilities.moveArm01(-200);
        utilities.wait(3000, telemetry);


    }


    private void buildTrajectories()
    {

        adjustment = drive.trajectoryBuilder((drive.getPoseEstimate()))
                .forward(31).build();

        //adjustment2 = drive.trajectoryBuilder(adjustment.end())
              //  .forward(26).build();


    }

}
