package org.firstinspires.ftc.team15021.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team15021.hardware.RavioliHardware;

@TeleOp(name = "Ravioli")
public class Ravioli extends OpMode
{

    // Initialization
    RavioliHardware hardware;
    final double FAST_SPEED = .8;
    final double SLOW_SPEED = .5;
    double slowConstant = SLOW_SPEED;
    boolean flip0 = false;
    boolean flip1 = false;
    ElapsedTime buttonTime = null;
    ElapsedTime buttonTime2 = null;
    ElapsedTime buttonTime3 = null;
    ElapsedTime buttonTime4 = null;





    public void init()
    {

        // Initialize Hardware
        hardware = new RavioliHardware();
        hardware.init(hardwareMap);
        buttonTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        buttonTime2 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        buttonTime3 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        buttonTime4 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


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
        if(!flip1)
            drive();
        else
            altdrive();
        moveArm();
        claw();
        if(gamepad1.triangle&&buttonTime4.time()>250)
        {
            flip1 = !flip1;
            buttonTime4.reset();
        }
    }

    private void drive()
    {
        // Mecanum drivecode
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = -gamepad1.left_stick_x; // Counteract imperfect strafing
        double rx = -gamepad1.right_stick_x;

        double leftFrontPower = y - x - rx;
        double leftRearPower = y + x - rx;
        double rightFrontPower = y + x + rx;
        double rightRearPower = y - x + rx;


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

        if (gamepad1.dpad_up )
        {
            leftFrontPower = .7;
            rightRearPower = .7;
            rightFrontPower = .7;
            leftRearPower = .7;
        }
        else if (gamepad1.dpad_down)
        {
            leftFrontPower = -.7;
            rightRearPower = -.7;
            rightFrontPower = -.7;
            leftRearPower = -.7;
        }
        else if (gamepad1.dpad_right)
        {
            leftFrontPower = .7;
            rightRearPower = .7;
            rightFrontPower = -.7;
            leftRearPower = -.7;
        }
        else if (gamepad1.dpad_left)
        {
            leftFrontPower = -.7;
            rightRearPower = -.7;
            rightFrontPower = .7;
            leftRearPower = .7;
        }


        if (gamepad1.square && slowConstant == FAST_SPEED && buttonTime3.time() >= 500)
        {
            slowConstant = SLOW_SPEED;
            buttonTime3.reset();
        }
        else if (gamepad1.square && slowConstant == SLOW_SPEED && buttonTime3.time() >= 500)
        {
            slowConstant = FAST_SPEED;
            buttonTime3.reset();
        }

        hardware.leftFront.setPower(leftFrontPower * slowConstant);
        hardware.leftRear.setPower(leftRearPower * slowConstant);
        hardware.rightFront.setPower(rightFrontPower * slowConstant);
        hardware.rightRear.setPower(rightRearPower * slowConstant);

    }
    public void altdrive()
    {
        double leftFrontPower = 0;
        double rightRearPower = 0;
        double rightFrontPower = 0;
        double leftRearPower = 0;
        if(gamepad1.dpad_up)
        {
            leftFrontPower = .2;
            rightRearPower = .2;
            rightFrontPower = .2;
            leftRearPower = .2;
        }
        if(gamepad1.dpad_down)
        {
            leftFrontPower = -.2;
            rightRearPower = -.2;
            rightFrontPower = -.2;
            leftRearPower = -.2;
        }
        if(gamepad1.dpad_right)
        {
            leftFrontPower = .2;
            rightRearPower = -.2;
            rightFrontPower = -.2;
            leftRearPower = .2;
        }
        if(gamepad1.dpad_left)
        {
            leftFrontPower = -.2;
            rightRearPower = .2;
            rightFrontPower = .2;
            leftRearPower = -.2;
        }
        hardware.leftFront.setPower(leftFrontPower);
        hardware.leftRear.setPower(leftRearPower);
        hardware.rightFront.setPower(rightFrontPower);
        hardware.rightRear.setPower(rightRearPower);

    }

    public void moveArm()
    {
        hardware.armMotor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.armMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.armMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.armMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        hardware.armMotor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.armMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.armMotor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            if(gamepad2.left_trigger>.1)
            {
                hardware.armMotor0.setPower(-gamepad2.left_trigger*.8);
                hardware.armMotor1.setPower(-gamepad2.left_trigger*.8);
                hardware.armMotor2.setPower(-gamepad2.left_trigger*.8);
            }
            else
            {
                hardware.armMotor0.setPower(gamepad2.right_trigger*.8);
                hardware.armMotor1.setPower(gamepad2.right_trigger*.8);
                hardware.armMotor2.setPower(gamepad2.right_trigger*.8);
            }



        hardware.armMotor3.setPower(gamepad2.left_stick_y*.7);
    }

    public void claw()
    {
        if (gamepad2.right_bumper&&!flip0&&buttonTime.time()>250)
        {
            hardware.servo1.setPosition(1);
            flip0 = true;
            buttonTime.reset();
        }
        else if (gamepad2.right_bumper&&flip0&&buttonTime.time()>250)
        {
            hardware.servo1.setPosition(0);
            flip0 = false;
            buttonTime.reset();
        }
        if (gamepad2.dpad_down&&buttonTime2.time()>250)
        {
            hardware.servo0.setPosition(0);
            buttonTime2.reset();
        }
        if (gamepad2.dpad_right&&buttonTime2.time()>250)
        {
            hardware.servo0.setPosition(.5);
            buttonTime2.reset();
        }
        if (gamepad2.dpad_up&&buttonTime2.time()>250)
        {
            hardware.servo0.setPosition(1);
            buttonTime2.reset();
        }

    }



}
