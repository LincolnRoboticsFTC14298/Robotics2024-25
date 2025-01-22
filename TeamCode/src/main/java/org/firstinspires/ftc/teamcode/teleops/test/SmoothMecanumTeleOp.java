package org.firstinspires.ftc.teamcode.teleops.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.test.MecanumDrive;

@TeleOp(name = "SmoothMecanumTeleOp", group = "TeleOp")
public class SmoothMecanumTeleOp extends LinearOpMode {

    private static final double SMOOTHING_FACTOR = 0.1;

    private double smoothedForward = 0;
    private double smoothedStrafe = 0;
    private double smoothedRotate = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            double rawForward = -gamepad1.left_stick_y;
            double rawStrafe = gamepad1.left_stick_x;
            double rawRotate = gamepad1.right_stick_x;

            rawForward = applyDeadZone(rawForward);
            rawStrafe = applyDeadZone(rawStrafe);
            rawRotate = applyDeadZone(rawRotate);

            smoothedForward = smoothInput(smoothedForward, rawForward);
            smoothedStrafe = smoothInput(smoothedStrafe, rawStrafe);
            smoothedRotate = smoothInput(smoothedRotate, rawRotate);

            double denominator = Math.max(Math.abs(smoothedForward) + Math.abs(smoothedStrafe) + Math.abs(smoothedRotate), 1);
            double frontLeftPower = (smoothedForward + smoothedStrafe + smoothedRotate) / denominator;
            double backLeftPower = (smoothedForward - smoothedStrafe + smoothedRotate) / denominator;
            double frontRightPower = (smoothedForward - smoothedStrafe - smoothedRotate) / denominator;
            double backRightPower = (smoothedForward + smoothedStrafe - smoothedRotate) / denominator;

            drive.setMotorPowers(frontLeftPower, backLeftPower, frontRightPower, backRightPower);

            telemetry.addData("Smoothed Forward", smoothedForward);
            telemetry.addData("Smoothed Strafe", smoothedStrafe);
            telemetry.addData("Smoothed Rotate", smoothedRotate);
            telemetry.addData("Front Left Power", frontLeftPower);
            telemetry.addData("Back Left Power", backLeftPower);
            telemetry.addData("Front Right Power", frontRightPower);
            telemetry.addData("Back Right Power", backRightPower);
            telemetry.update();
        }
    }

    private double smoothInput(double previousValue, double currentValue) {
        return previousValue + (currentValue - previousValue) * SMOOTHING_FACTOR;
    }

    private double applyDeadZone(double input) {
        return Math.abs(input) > 0.05 ? input : 0;
    }
}
