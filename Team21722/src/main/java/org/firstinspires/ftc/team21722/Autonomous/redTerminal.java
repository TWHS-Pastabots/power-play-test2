package org.firstinspires.ftc.team21722.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;


import org.firstinspires.ftc.team21722.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team21722.Hardware.MacaroniHardware;


@Config
@Autonomous(name = "redTerminal")
public class redTerminal extends LinearOpMode
{

    SampleMecanumDrive drive;
    private Pose2d posTerminal = new Pose2d(-10,-23.5,0 );
    private Utilities utilities;
    private Object MacaroniHardware;
    private Trajectory terminal;

    @Override
    public void runOpMode() throws InterruptedException
    {
        drive= new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-45, -65, 0));
        buildTrajectories();

        MacaroniHardware hardware = new MacaroniHardware();
        utilities = new Utilities(hardware);
        hardware.init(hardwareMap);

        utilities.closeClaw();
        waitForStart();



        drive.followTrajectory(terminal);





    }

    private void buildTrajectories()
    {
        terminal = drive.trajectoryBuilder(drive.getPoseEstimate(), Math.toRadians(0))
                .back(-20)
                .build();



    }
}
