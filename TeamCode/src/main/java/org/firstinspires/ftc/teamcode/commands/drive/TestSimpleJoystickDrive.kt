package org.firstinspires.ftc.teamcode.commands.drive

import com.acmerobotics.roadrunner.PoseVelocity2d
import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor
import org.firstinspires.ftc.teamcode.teleops.SimpleTeleOp

class TestSimpleJoystickDrive(
        private val mecanum: MecanumDrive,
        private val input: () -> PoseVelocity2d,
        private val fieldCentric: () -> Boolean,
        private val hwMap: HardwareMap,
        private val voltageSensor: VoltageSensor,
        private val liftPower: () -> Double,
        private val telemetry: Telemetry,
        private val clawInput: () -> SimpleTeleOp.TwoStateInput,
        private val passthroughInput: () -> SimpleTeleOp.TwoStateInput,

        ) : CommandBase() {
    val CLAW_CLOSED = 0.05
    val CLAW_OPEN = 0.15
    val PASSTHROUGH_EXTENDED = 0.6
    val PASSTHROUGH_RETRACTED = 0.01

    val liftLeft: DcMotorEx = hwMap.get(DcMotorEx::class.java, "liftLeft")
    val liftRight: DcMotorEx = hwMap.get(DcMotorEx::class.java, "liftRight")
    val claw: Servo = hwMap.get(Servo::class.java, "claw")
    val passThroughLeft: Servo = hwMap.get(Servo::class.java, "left")
    val passThroughRight: Servo = hwMap.get(Servo::class.java, "right")

    var clawPos = CLAW_CLOSED
    var passthroughPos = PASSTHROUGH_RETRACTED

    init{
        addRequirements(mecanum)
        liftLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        liftRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        passThroughRight.direction = Servo.Direction.REVERSE
    }

    override fun execute() {
        //drive
        val drivePower =
            if (fieldCentric.invoke())
                mecanum.pose.inverse() * input.invoke()
            else
                input.invoke()
        mecanum.setDrivePowers(drivePower)

        //lift
        liftLeft.power = liftPower.invoke()
        liftRight.power = liftPower.invoke()
        telemetry.addData("Lift Power", liftPower.invoke())
        //telemetry.addData("Encoder Position", pos)

        //claw
        if (clawInput.invoke() == SimpleTeleOp.TwoStateInput.OPEN) {
            clawPos = CLAW_OPEN
        } else if (clawInput.invoke() == SimpleTeleOp.TwoStateInput.CLOSED) {
            clawPos = CLAW_CLOSED
        }
        claw.position = clawPos
        telemetry.addData("Claw Position", clawPos)

        //passthrough
        if (passthroughInput.invoke() == SimpleTeleOp.TwoStateInput.OPEN) {
            passthroughPos = PASSTHROUGH_EXTENDED
        } else if (passthroughInput.invoke() == SimpleTeleOp.TwoStateInput.CLOSED) {
            passthroughPos = PASSTHROUGH_RETRACTED
        }
        passThroughLeft.position = passthroughPos
        passThroughRight.position = passthroughPos
        telemetry.addData("Passthrough Position", passthroughPos)
    }
}