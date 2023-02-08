package org.firstinspires.ftc.team16909.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.team16909.hardware.FettucineHardware;

@TeleOp(name = "Fettucine")
public class Fettucine extends OpMode
{
    // Initialization
    FettucineHardware hardware;
    final double FAST_SPEED = .8;
    final double SLOW_SPEED = .35;
    double slowConstant = FAST_SPEED;
    ElapsedTime grabTime = null;
    ElapsedTime buttonTime = null;
    ElapsedTime grabButtonTime = null;
    int buttonCounter = 0;

    public void init()
    {
        // Initialize Hardware
        hardware = new FettucineHardware();
        hardware.init(hardwareMap);
        buttonTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        grabTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        grabButtonTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void start()
    {
        telemetry.addData("Status", "Started");
        telemetry.update();
    }
    public void loop()
    {
        drive();
        lift();
        arm();
    }

    private void drive()
    {
        // Mecanum drivecode
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x; // Counteract imperfect strafing
        double rx = -gamepad1.right_stick_x;

        double leftFrontPower = y + x + rx;
        double leftRearPower = y - x + rx;
        double rightFrontPower = y - x - rx;
        double rightRearPower = y + x - rx;


        if (Math.abs(leftFrontPower) > 1 || Math.abs(leftRearPower) > 1 ||
                Math.abs(rightFrontPower) > 1 || Math.abs(rightRearPower) > 1 )
        {
            // Find the largest power
            double max;
            max = Math.max(Math.abs(leftFrontPower), Math.abs(leftRearPower));
            max = Math.max(Math.abs(rightFrontPower), max);
            max = Math.max(Math.abs(rightRearPower), max);

            // Divide everything by max (it's positive so we don't need to worry
            // about signs)
            leftFrontPower /= max;
            leftRearPower /= max;
            rightFrontPower /= max;
            rightRearPower /= max;
        }

        if (gamepad1.dpad_up)
        {
            leftFrontPower = slowConstant;
            rightRearPower = slowConstant;
            rightFrontPower = slowConstant;
            leftRearPower = slowConstant;
        }
        else if (gamepad1.dpad_down)
        {
            leftFrontPower = -slowConstant;
            rightRearPower = -slowConstant;
            rightFrontPower = -slowConstant;
            leftRearPower = -slowConstant;
        }
        else if (gamepad1.dpad_left)
        {
            leftFrontPower = -1;
            rightRearPower = -1;
            rightFrontPower = 1;
            leftRearPower = 1;
        }
        else if (gamepad1.dpad_right)
        {
            leftFrontPower = 1;
            rightRearPower = 1;
            rightFrontPower = -1;
            leftRearPower = -1;
        }

        if (gamepad1.square && slowConstant == FAST_SPEED && buttonTime.time() >= 500)
        {
            slowConstant = SLOW_SPEED;
            buttonTime.reset();
        }
        else if (gamepad1.square && slowConstant == SLOW_SPEED && buttonTime.time() >= 500)
        {
            slowConstant = FAST_SPEED;
            buttonTime.reset();
        }

        hardware.leftFront.setPower(leftFrontPower * slowConstant);
        hardware.leftRear.setPower(leftRearPower * slowConstant);
        hardware.rightFront.setPower(rightFrontPower * slowConstant);
        hardware.rightRear.setPower(rightRearPower * slowConstant);
    }

    public void lift()
    {
        if (gamepad2.dpad_up)
        {
            hardware.liftMotorOne.setPower(1); //sid was here
            hardware.liftMotorTwo.setPower(.905);
        }

        else if (gamepad2.dpad_down)
        {
            hardware.liftMotorOne.setPower(-1);
            hardware.liftMotorTwo.setPower(-.905); //sid was here
        }
        else
        {
            hardware.liftMotorOne.setPower(0.0);
            hardware.liftMotorTwo.setPower(0.0); //sid was here
        }
    }

    public void arm()
    {
        if (gamepad2.triangle) //rotates grabber
        {
            hardware.armServoOne.setPower(-1);
            hardware.armServoTwo.setPower(1);
        }
        else if (gamepad2.cross)
        {
            hardware.armServoOne.setPower(1); //sid was here
            hardware.armServoTwo.setPower(-1);
        }
        else
        {
            hardware.armServoOne.setPower(0.0);   //sid was here
            hardware.armServoTwo.setPower(0.0);
        }

        if (gamepad2.right_bumper && buttonTime.time() >= 500) //manual intake + outtake
        {
            if (buttonCounter == 0)
            {
                buttonCounter = 1;
            }
            else
            {
                buttonCounter = 0;
            }
            hardware.grabberServo.setPosition(buttonCounter);
            telemetry.addData("pos", buttonCounter);
            buttonTime.reset();
        }

        if (gamepad2.left_bumper && grabButtonTime.time() >= 500)
        {
            grabButtonTime.reset();
            hardware.grabberServo.setPosition(0);
            grabTime.reset();
            while (grabTime.time() < 250)
            {
                continue;
            }
            hardware.grabberServo.setPosition(1);
        }

        if(gamepad2.right_trigger > 0.0) //going from front to back (not purple to purple)
        {
            hardware.armMotorOne.setPower(gamepad2.right_trigger);
            //hardware.armServoTwo.setPower(-0.1);
        }
        else if (gamepad2.left_trigger > 0.0) //going from back to front (purple to not purple)
        {
            hardware.armMotorOne.setPower(-gamepad2.left_trigger);
            //hardware.armServoTwo.setPower(-0.1);
        }
        else
        {
            hardware.armMotorOne.setPower(0);
        }
    }
}
