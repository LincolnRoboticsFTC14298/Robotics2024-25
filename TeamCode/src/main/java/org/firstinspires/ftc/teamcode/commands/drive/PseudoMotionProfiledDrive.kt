package org.firstinspires.ftc.teamcode.commands.drive

import android.util.Log
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive
import org.firstinspires.ftc.teamcode.util.normalize
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

/**
 * Input in robot (tangent) frame
 */
class PseudoMotionProfiledDrive(
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
        val currentVel = mecanum.robotVelRobot.linearVel.norm()

        var desiredInput = input.invoke()
        if (isInputVelocityNormalized) desiredInput = PoseVelocity2d(desiredInput.linearVel * MecanumDrive.MAX_WHEEL_VEL, desiredInput.angVel)
        if (isInputRotationNormalized) desiredInput = PoseVelocity2d(desiredInput.linearVel, desiredInput.angVel * MecanumDrive.MAX_ANG_VEL)

        // Get the time from the last loop
        val dt = timer.seconds()

        // Calculate the maximum velocity it can drive before needing to stop
        val maxVelToStop = sqrt(-2 * MecanumDrive.MIN_PROFILE_ACCEL * distanceFromTarget.invoke()) // TODO update to account for tangential driving

        // Calculate the maximum and minimum velocity that the drivetrain could possibly travel
        val maxVelFromLast = currentVel + MecanumDrive.MAX_PROFILE_ACCEL * dt
        val minVelFromLast = currentVel + MecanumDrive.MIN_PROFILE_ACCEL * dt

        // Choose the minimum possible speed and obtain the max of the physically possible velocity
        val velocity = max(minVelFromLast, minOf(maxVelFromLast, desiredInput.linearVel.norm(), maxVelToStop))

        //TODO maybe change to min(maxrot with accel, maxvel) to prevent maxRotFromLast from exceeding MAX_ANG_VEL?
        val maxRotFromLast = mecanum.robotVelRobot.angVel + MecanumDrive.MAX_ANG_ACCEL * dt
        val minRotFromLast = mecanum.robotVelRobot.angVel - MecanumDrive.MAX_ANG_ACCEL * dt

        val desiredRotVel = desiredInput.angVel * MecanumDrive.MAX_ANG_VEL
        val immediateVelOffsetCorrection = (desiredRotVel - mecanum.robotVelRobot.angVel)

        //val rotation = max(minRotFromLast, min(maxRotFromLast, desiredInput.angVel)) // May cause issues w/ [ApproachRelativePoint] and [ApproachAngle]
        val rotation = max(minRotFromLast, min(maxRotFromLast, immediateVelOffsetCorrection)) // May cause issues w/ [ApproachRelativePoint] and [ApproachAngle]

        //telemetry?.addData("setDriveSignal", PoseVelocity2d(desiredInput.linearVel.normalize() * velocity, rotation))
        //telemetry?.update()

        Log.i("dt-------------------", dt.toString())
        Log.i("desiredRotVel", desiredRotVel.toString())
        Log.i("immediateVelOffsetCorrection", immediateVelOffsetCorrection.toString())
        Log.i("minRotFromLast", minRotFromLast.toString())
        Log.i("robotVelRobot.angVel", mecanum.robotVelRobot.angVel.toString())
        Log.i("maxRotFromLast", maxRotFromLast.toString())
        Log.i("rotation", rotation.toString())

        mecanum.setDriveSignal(PoseVelocity2d(desiredInput.linearVel.normalize() * velocity, rotation))
        timer.reset()
    }

    override fun isFinished(): Boolean {
        return distanceFromTarget.invoke() <= maxTolerableDistance
    }

}
