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

@Autonomous(name = "HighJunction")

public class HighJunction extends LinearOpMode
{
    private SampleMecanumDrive drive;
    private Pose2d rightStart = new Pose2d(-36, 64, Math.toRadians(-90));
    //private Pose2d rightToMiddle = new Pose2d(-24, 34, Math.toRadians(-90));
    //private Pose2d rightJunction = new Pose2d(4, 34, Math.toRadians(-90));

    int liftPos1 = 3897;
    int armPos1 = 18;

    private TrajectorySequence traj1, traj2;

    @Override
    public void runOpMode() throws InterruptedException
    {
        FettucineHardware hardware = new FettucineHardware();

        hardware.init(hardwareMap);

        hardware.leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        hardware.leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        hardware.rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        hardware.rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        hardware.armMotorOne.setDirection(DcMotorSimple.Direction.FORWARD);


        Utilities utilities = new Utilities(hardware);

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(rightStart);

        buildTrajectories();

        utilities.intake();

        waitForStart();

        if (!opModeIsActive())
        {
            return;
        }

        //utilities.moveArm(armPos1);
        //utilities.wait(5000);

        //utilities.moveLift(liftPos1);
        //utilities.wait(5000);

        drive.followTrajectorySequence(traj1);

        utilities.wait(1000);

        utilities.wait(500, telemetry);

        utilities.outtake();
    }

    public void buildTrajectories()
    {
        traj1 = drive.trajectorySequenceBuilder(rightStart)
                .forward(4)
                .waitSeconds(1)
                .turn(Math.toRadians(90))
                .waitSeconds(1)
                .forward(4)
                .waitSeconds(1)
                .turn(Math.toRadians(-90))
                .waitSeconds(1)
                .forward(4)
                .waitSeconds(1)
                .turn(Math.toRadians(40))
                .build();
    }
}
