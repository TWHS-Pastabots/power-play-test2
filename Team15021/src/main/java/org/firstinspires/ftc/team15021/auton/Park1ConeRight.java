/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.team15021.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team15021.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team15021.hardware.RavioliHardware;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@Config
@Autonomous(name = "Park1ConeRight")
public class Park1ConeRight extends LinearOpMode
{

    AprilTagPipeline aprilTagPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    SampleMecanumDrive drive;

    private Trajectory spline1;
    private Trajectory adjust1;
    private Trajectory linear1;
    private Trajectory extraCone;
    private Trajectory adjust2;
    private Trajectory scoreTwo;
    private Trajectory reset;
    private Trajectory park1;
    private Trajectory park2;
    private Trajectory park3;



    private Pose2d midCone = new Pose2d(28, 3, Math.toRadians(50));
    private Pose2d interim1 = new Pose2d(22, -4.25, Math.toRadians(0));
    private Pose2d adj1 = new Pose2d(42, -4.25, Math.toRadians(0));
    private Pose2d interim2 = new Pose2d(1.5, 0 ,0);
    private Pose2d interim3;
    private Pose2d stack;
    private Pose2d midCone2;
    private Pose2d resetPos;



    OpenCvInternalCamera webcam;
    SignalPipeline pipeline;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int one = 4; // Tag ID 4 from the 36h11 family
    int two = 8; // Tag ID 8 from the 36h11 family
    int three = 12; // Tag ID 12 from the 36h11 family


    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        aprilTagPipeline = new AprilTagPipeline(tagsize, fx, fy, cx, cy);

        webcam.setPipeline(aprilTagPipeline);
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

        buildTrajectories();
        utilities.closeClaw();
        utilities.resetClaw();

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == one || tag.id == two || tag.id == three)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest==null||tagOfInterest.id == one)
        {
            drive.followTrajectory(adjust2);
            utilities.moveArm012(270);
            utilities.moveArm3(130);
            utilities.tiltMid();
            utilities.wait(1000, telemetry);
            drive.followTrajectory(spline1);
            utilities.openClaw();
            drive.followTrajectory(adjust1);
            drive.turn(Math.toRadians(90));
            drive.followTrajectory(park1);
            utilities.moveArm3(-125);
            utilities.wait(2000, telemetry);
        }
        else if(tagOfInterest.id == two)
        {
            drive.followTrajectory(adjust2);
            utilities.moveArm012(270);
            utilities.moveArm3(130);
            utilities.tiltMid();
            utilities.wait(1000, telemetry);
            drive.followTrajectory(spline1);
            utilities.openClaw();
            drive.followTrajectory(adjust1);
            drive.turn(Math.toRadians(90));
            drive.followTrajectory(park2);
            utilities.moveArm3(-125);
            utilities.wait(2000, telemetry);
        }
        else if(tagOfInterest.id == three)
        {
            drive.followTrajectory(adjust2);
            utilities.moveArm012(270);
            utilities.moveArm3(130);
            utilities.tiltMid();
            utilities.wait(1000, telemetry);
            drive.followTrajectory(spline1);
            utilities.openClaw();
            drive.followTrajectory(adjust1);
            drive.turn(Math.toRadians(90));
            drive.followTrajectory(park3);
            utilities.moveArm3(-125);
            utilities.wait(2000, telemetry);
        }

    }

    private void buildTrajectories()
    {

        adjust2 = drive.trajectoryBuilder((drive.getPoseEstimate()))
                .lineToLinearHeading(interim2)
                .build();

        spline1 = drive.trajectoryBuilder(adjust2.end(), 0)
                .splineToLinearHeading(midCone, 0)
                .build();

        adjust1 = drive.trajectoryBuilder(spline1.end(), Math.toRadians(180))
                .splineToLinearHeading(interim1, Math.toRadians(180))
                .build();

        linear1 = drive.trajectoryBuilder(adjust1.end())
                .lineToLinearHeading(adj1)
                .build();

        extraCone = drive.trajectoryBuilder(new Pose2d(42, -4.25, Math.toRadians(-215)))
                .forward(1)
                .build();

        park1 = drive.trajectoryBuilder(new Pose2d(22, -4.25, Math.toRadians(90)))
                .forward(28)
                .build();

        park2 = drive.trajectoryBuilder(new Pose2d(22, -4.25, Math.toRadians(90)))
                .back(.25)
                .build();

        park3 = drive.trajectoryBuilder(new Pose2d(22, -4.25, Math.toRadians(90)))
                .back(24)
                .build();



    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
