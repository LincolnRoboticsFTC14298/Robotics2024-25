package org.firstinspires.ftc.teamcode.subsystems

import android.util.Log
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.SimpleServo
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.Range.clip
import org.firstinspires.ftc.teamcode.FieldConfig

/**
 * Claw subsystem consists of a servo, potentiometer and a color sensor.
 * @param hwMap HardwareMap.
 */
@Config
class Bucket(hwMap: HardwareMap) : SubsystemBase() {

    private val servo = SimpleServo(hwMap, bucketServoName, 0.0, 360.0)

    private var setpoint: Double = 0.0
        set(position) {
            field = clip(position, 0.0, 1.0)
            Log.i("Bucket desired position", setpoint.toString())
        }

    init {
        setpoint = closedBucketPosition
        servo.position = closedBucketPosition
    }

    override fun periodic() {
        Log.v("Bucket estimated angle", getPositionEstimate().toString())
        //servo.position = motionProfile[timer.seconds()].value()
        servo.position = setpoint
    }

    /**
     * Open to pick up the cone
     */
    fun dump() {
            setpoint = openBucketPosition
    }

    private fun getPositionEstimate() : Double {
        return servo.position
    }


    /**
     * For debugging/tuning purposes
     */
    fun fetchTelemetry(packet: TelemetryPacket) {
        packet.put("Position Estimate", getPositionEstimate())
        packet.put("Desired position", setpoint)
    }

    fun drawClaw(canvas: Canvas, clawOffset: Vector2d, pose: Pose2d) {
        canvas.setStroke("#8CA231")
        val (x, y) = pose.position.plus(
            pose.heading.inverse().times(
                clawOffset
            )
        )
        canvas.fillCircle(x, y, FieldConfig.spikeDiameter / 2.0 * 1.2)
    }

    companion object {
        const val bucketServoName = "bucket"

        @JvmField
        var closedBucketPosition = 0.75
        @JvmField
        var openBucketPosition = 0.95


    }

}