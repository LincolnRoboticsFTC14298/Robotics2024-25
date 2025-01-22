package org.firstinspires.ftc.teamcode.teleops

import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.drive.SimpleJoystickDrive
import org.firstinspires.ftc.teamcode.subsystems.DualClaw
import org.firstinspires.ftc.teamcode.subsystems.Bucket
import org.firstinspires.ftc.teamcode.subsystems.Lift
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.Passthrough
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometryLocalizer
import org.firstinspires.ftc.teamcode.subsystems.localization.StartingPoseStorage

@TeleOp
class MainTeleOp  : CommandOpMode() {
    override fun initialize() {
        /****************************************************
         * Initialize hardware                              *
         ****************************************************/

        val voltageSensor = VoltageSensor(hardwareMap)
        val lift = Lift(hardwareMap, voltageSensor)
        val claw = DualClaw(hardwareMap)
        val bucket = Bucket(hardwareMap)
        val passthrough = Passthrough(hardwareMap)
        //val vision = Vision(hardwareMap)
        //val localizer = MecanumMonteCarloLocalizer(hardwareMap, vision, Pose2d(), arrayToRowMatrix(doubleArrayOf()))
        val localizer = OdometryLocalizer(hardwareMap)
        val mecanum = MecanumDrive(hardwareMap, StartingPoseStorage.startingPose.pose, localizer, voltageSensor)

        //register(lift, claw, passthrough, mecanum, vision)

        val hubs: List<LynxModule> = hardwareMap.getAll(LynxModule::class.java)

        for (hub in hubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }

        register(voltageSensor, mecanum, lift, claw, passthrough, bucket)

        /****************************************************
         * Driver 1 Controls                                *
         * Driving and semi autonomous control              *
         ****************************************************/

        val driver1 = GamepadEx(gamepad1)

        /**
         * Drive
         */

        val scaleFactorProvider = { if (lift.isRetracted) 1.0 else 0.3 }

        val input = { PoseVelocity2d(Vector2d(driver1.leftY * scaleFactorProvider.invoke(), -driver1.leftX * scaleFactorProvider.invoke()), -driver1.rightX * 0.5 * (scaleFactorProvider.invoke() * 1.5)) }

        var fieldCentric = true
        val fieldCentricProvider = { fieldCentric }

        var autoOpen: Boolean = true

        mecanum.defaultCommand = SimpleJoystickDrive(mecanum, input, fieldCentricProvider)

        /**
         * Claw
         */
    /*    driver1
                .getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(
                        InstantCommand({if(passthrough.passthroughState != Passthrough.PassthroughState.DEPOSIT && !passthrough.isInTransit()) claw.incramentOpen()}, claw),
                )

        driver1
                .getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(
                        InstantCommand({if(passthrough.passthroughState != Passthrough.PassthroughState.DEPOSIT) claw.incramentClosed()}, claw)
                )


        //drive mode
        driver1
                .getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(
                        InstantCommand({
                            if ((passthrough.passthroughState.ordinal + 1) < Passthrough.PassthroughState.values().size && lift.isRetracted) {
                                if (passthrough.passthroughState == Passthrough.PassthroughState.HALFWAY) {
                                    claw.close()
                                    autoOpen = true
                                }
                                passthrough.setState(Passthrough.PassthroughState.values()[passthrough.passthroughState.ordinal + 1])
                            }
                        }, passthrough)
                )
        driver1
                .getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(
                        InstantCommand({
                            if ((passthrough.passthroughState.ordinal - 1) >= 0 && lift.isRetracted) {
                                if (passthrough.passthroughState == Passthrough.PassthroughState.DEPOSIT) {
                                    claw.close()
                                }
                                if (passthrough.passthroughState == Passthrough.PassthroughState.HALFWAY && autoOpen == true && !passthrough.isInTransit()) {
                                    claw.open()
                                    autoOpen = false
                                }
                                passthrough.setState(Passthrough.PassthroughState.values()[passthrough.passthroughState.ordinal - 1])
                            }
                        }, passthrough)
                )


        //Pickup/Desposit
        driver1
                .getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(
                        SequentialCommandGroup(
                                InstantCommand(claw::close, claw),
                                WaitCommand(200),
                                InstantCommand(passthrough::deposit, passthrough),
                                WaitCommand(300),
                                InstantCommand({ lift.setHeightLastPos() }, lift)
                        )
                )
        driver1
                .getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER) //TODO Change to just passthrough/claw control
                .whenPressed(
                        SequentialCommandGroup(
                                InstantCommand(claw::close, claw),
                                InstantCommand(lift::retract, lift)
                        )
                )

        //lift heights
        driver1
                .getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(
                        SequentialCommandGroup(
                                InstantCommand(claw::close, claw),
                                WaitCommand(200),
                                InstantCommand(passthrough::deposit, passthrough),
                                WaitCommand(300),
                                InstantCommand({ lift.setHeight(Lift.LiftPosition.LOW) }, lift)
                        )
                )

        driver1
                .getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(
                        SequentialCommandGroup(
                                InstantCommand(claw::close, claw),
                                WaitCommand(200),
                                InstantCommand(passthrough::deposit, passthrough),
                                WaitCommand(300),
                                InstantCommand({ lift.setHeight(Lift.LiftPosition.MIDDLE) }, lift)
                        )
                )

        driver1
                .getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(
                        SequentialCommandGroup(
                                InstantCommand(claw::close, claw),
                                WaitCommand(200),
                                InstantCommand(passthrough::deposit, passthrough),
                                WaitCommand(300),
                                InstantCommand({ lift.setHeight(Lift.LiftPosition.HIGH) }, lift)
                        )
                )

        //release
        Trigger{ driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= 0.5 }
                .whenActive(InstantCommand({ if (!lift.isRetracted && !passthrough.isInTransit()) claw.releaseFirst() }, claw))

        Trigger { driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.5 }//TriggerReader(driver1, GamepadKeys.Trigger.RIGHT_TRIGGER)::wasJustPressed)
                .whenActive(InstantCommand({ if (!lift.isRetracted && !passthrough.isInTransit()) claw.releaseSecond() }, claw))

*/

        driver1
                .getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(
                        InstantCommand(bucket::dump, bucket)
                )
/*
        driver1
                .getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(
                        InstantCommand({ fieldCentric = !fieldCentric })
                )
*/
    }

}