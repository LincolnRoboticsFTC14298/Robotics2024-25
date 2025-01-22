package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class LiftTestTeleOp extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        //motors
        DcMotor motorLeft = hardwareMap.dcMotor.get("liftLeft");
        DcMotor motorRight = hardwareMap.dcMotor.get("liftRight"); //TODO Rename in config

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive())
        {
            motorLeft.setPower(gamepad1.left_stick_y/3.0);
            motorRight.setPower(gamepad1.left_stick_y/3.0);

        }
    }
}