package org.firstinspires.ftc.teamcode.subsystems.localization

import com.acmerobotics.roadrunner.Pose2d
import java.lang.Math.toRadians

enum class StartingPose(val pose: Pose2d) {
    BLUE_RIGHT(Pose2d(-63.0, -36.0, toRadians(-180.0))),
    BLUE_LEFT(Pose2d(-63.0, 12.0, toRadians(-180.0))),
    RED_LEFT(Pose2d(63.0, -36.0, toRadians(0.0))),
    RED_RIGHT(Pose2d(63.0, 12.0, toRadians(0.0)));

    fun isRedAlliance(): Boolean {
        return this == RED_LEFT || this == RED_RIGHT
    }
}

object StartingPoseStorage {
    @JvmStatic
    var startingPose: StartingPose = StartingPose.BLUE_RIGHT
}