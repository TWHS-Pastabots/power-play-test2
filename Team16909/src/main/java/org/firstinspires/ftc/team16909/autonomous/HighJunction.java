package org.firstinspires.ftc.team16909.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team16909.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16909.hardware.FettucineHardware;
import org.firstinspires.ftc.team16909.teleop.Fettucine;
@Autonomous
public class HighJunction extends LinearOpMode
{
    private SampleMecanumDrive drive;
    ElapsedTime waitTime = null;
    FettucineHardware hardware;

    public void runOpMode() throws InterruptedException
    {
        waitTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        hardware = new FettucineHardware();
        hardware.init(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();
        waitTime.reset();
        for (DcMotorEx motor : hardware.driveMotors)
        {
            motor.setPower(.65);
        }
        while (waitTime.time() < 1000)
        {
        }
        for (DcMotorEx motor : hardware.driveMotors)
        {
            motor.setPower(0.0);
        }
    }
}
