package org.firstinspires.ftc.teamcode.commands.drive

import com.acmerobotics.roadrunner.PoseVelocity2d
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive


/**
 * Manual joystick control of drivetrain.
 */
class MotionProfiledJoystickDrive2(
        mecanum: MecanumDrive,
        input: () -> PoseVelocity2d,
        fieldCentric: () -> Boolean,
        obstacleAvoidance: () -> Boolean = {false},
        var telemetry: Telemetry? = null
) : SequentialCommandGroup() {

    init {
        val rotatedInput = { mecanum.pose.inverse() * input.invoke() }
        val localInput = if (fieldCentric.invoke()) rotatedInput else input

        addCommands(
            PseudoMotionProfiledDrive2(
                mecanum,
                localInput,
                isInputVelocityNormalized = true,
                isInputRotationNormalized = true,
                telemetry = telemetry
            )
        )
        addRequirements(mecanum)
    }

    override fun isFinished(): Boolean {
        return false
    }

    fun calculateForce() {

//        if (obstacleAvoidance.invoke()) {
//            var avoidanceForce = Vector2d(0.0, 0.0)
//
//            for (x in -2..2) {
//                for (y in -2..2) {
//                    val corner = Vector2d(x * RobotConfig.tileSize, y * RobotConfig.tileSize)
//                    val diff = poseEstimate.vec().minus(corner)
//                    val dist = diff.norm()
//                    val k = 0.01 // TODO put in config
//                    avoidanceForce += diff * k / (dist * dist * dist)
//                }
//            }
//
//            avoidanceForce = avoidanceForce.rotated(-poseEstimate.heading)
//            fieldFramePower += Vector2d(fieldFrameInput.x * avoidanceForce.x, fieldFrameInput.y * avoidanceForce.y)
//        }

    }

}