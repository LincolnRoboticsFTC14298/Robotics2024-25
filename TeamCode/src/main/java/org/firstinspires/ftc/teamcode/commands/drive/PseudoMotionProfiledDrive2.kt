package org.firstinspires.ftc.teamcode.commands.drive

import android.util.Log
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive
import org.firstinspires.ftc.teamcode.util.normalize
import java.lang.Double.max
import kotlin.math.abs
import kotlin.math.min

/**
 * Input in robot (tangent) frame
 */
class PseudoMotionProfiledDrive2(
        private val mecanum: MecanumDrive,
        private val input: () -> PoseVelocity2d,
        private val isInputVelocityNormalized: Boolean,
        private val isInputRotationNormalized: Boolean,
        private val distanceFromTarget: () -> Double = { Double.MAX_VALUE },
        private val maxTolerableDistance: Double = 1.0, // in
        var telemetry: Telemetry? = null
) : CommandBase() {

    val timer = ElapsedTime()

    init {
        timer.reset()
        addRequirements(mecanum)
    }

    override fun execute() {
        mecanum.periodic()

        val currentVel = mecanum.robotVelRobot

        var targetVel = input.invoke()
        if (isInputVelocityNormalized) targetVel = PoseVelocity2d(targetVel.linearVel * MecanumDrive.MAX_WHEEL_VEL, targetVel.angVel)
        if (isInputRotationNormalized) targetVel = PoseVelocity2d(targetVel.linearVel, targetVel.angVel * MecanumDrive.MAX_ANG_VEL)

        // Get the time from the last loop
        val dt = timer.seconds()
        timer.reset()

        var targetLinearAccel = (targetVel.linearVel - currentVel.linearVel) / dt
        if (targetLinearAccel.norm() > MecanumDrive.MAX_PROFILE_ACCEL) {
            targetLinearAccel = targetLinearAccel.normalize() * MecanumDrive.MIN_PROFILE_ACCEL
        }

        var targetAngularAccel = (targetVel.angVel - currentVel.angVel) / dt
        targetAngularAccel = max(min(targetAngularAccel, MecanumDrive.MAX_ANG_ACCEL), -MecanumDrive.MAX_ANG_ACCEL)

        var setVelX = currentVel.linearVel.x + targetLinearAccel.x * dt
        var setVelY = currentVel.linearVel.y + targetLinearAccel.y * dt
        var setVelAng = currentVel.angVel + targetAngularAccel * dt

        if (abs(setVelX) < 0.2) {setVelX = 0.0}
        if (abs(setVelY) < 0.2) {setVelY = 0.0}
        if (abs(setVelAng) < 0.2) {setVelAng = 0.0}

        Log.i("targetVel", targetVel.toString())
        Log.i("targetLinearAccel", targetLinearAccel.toString())
        Log.i("targetAngularAccel", targetAngularAccel.toString())
        Log.i("currentVel", currentVel.toString())

        mecanum.setDriveSignal(PoseVelocity2d(Vector2d(setVelX, setVelY), setVelAng))
    }

    override fun isFinished(): Boolean {
        return distanceFromTarget.invoke() <= maxTolerableDistance
    }

}
