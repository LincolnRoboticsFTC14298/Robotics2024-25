package org.firstinspires.ftc.teamcode.commands.drive

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Vector2d
import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.localization.StartingPoseStorage

class SimpleJoystickDrive(
        private val mecanum: MecanumDrive,
        private val input: () -> PoseVelocity2d,
        private val fieldCentric: () -> Boolean,
) : CommandBase() {

    init{
        addRequirements(mecanum)
    }

    override fun execute() {
        val power =
            if (fieldCentric.invoke() && StartingPoseStorage.startingPose.isRedAlliance()) //TODO TEST
                mecanum.pose.inverse() * Pose2d(Vector2d(0.0, 0.0), Rotation2d.exp(Math.PI)) * input.invoke()
            else if (fieldCentric.invoke() && !StartingPoseStorage.startingPose.isRedAlliance()) {
                mecanum.pose.inverse() * input.invoke()
            }
            else
                input.invoke()
        mecanum.setDrivePowers(power)
    }

}