package org.firstinspires.ftc.teamcode.teleops

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.drive.TestSimpleJoystickDrive
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometryLocalizer

@TeleOp
class SimpleTeleOp : CommandOpMode() {

    enum class TwoStateInput {
        OPEN,
        CLOSED,
        NO_INPUT
    }

    override fun initialize() {

        /****************************************************
         * Initialize hardware                              *
         ****************************************************/

        val voltageSensor = VoltageSensor(hardwareMap)
        val localizer = OdometryLocalizer(hardwareMap)
        val mecanum = MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, Math.toRadians(90.0)), localizer, voltageSensor, telemetry)

        register(mecanum)

        val driver1 = GamepadEx(gamepad1)


        /**
         * Drive
         */
        val input = { PoseVelocity2d(Vector2d(driver1.leftY, -driver1.leftX), -driver1.rightX) }
        val liftPower = {
            if (driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2) {
                -0.5
            } else if (driver1.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                0.5
            } else {
                0.0
            }
        }
        val clawInput = {
            if (driver1.getButton(GamepadKeys.Button.B)) {
                TwoStateInput.OPEN
            } else if (driver1.getButton(GamepadKeys.Button.A)) {
                TwoStateInput.CLOSED
            } else {
                TwoStateInput.NO_INPUT
            }
        }
        val passthroughInput = {
            if (driver1.getButton(GamepadKeys.Button.Y)) {
                TwoStateInput.OPEN
            } else if (driver1.getButton(GamepadKeys.Button.X)) {
                TwoStateInput.CLOSED
            } else {
                TwoStateInput.NO_INPUT
            }
        }

        var fieldCentric = false

        mecanum.defaultCommand =
            TestSimpleJoystickDrive(
                mecanum,
                input,
                { fieldCentric },
                hardwareMap,
                voltageSensor,
                liftPower,
                telemetry,
                clawInput,
                passthroughInput)
    }

    override fun run() {
        super.run()
        telemetry.update()

    }

}
