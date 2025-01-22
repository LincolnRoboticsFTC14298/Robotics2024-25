package org.firstinspires.ftc.teamcode.subsystems

import android.util.Log
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.SimpleServo
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.Range.clip

/**
 * Claw subsystem consists of a servo, potentiometer and a color sensor.
 * @param hwMap HardwareMap.
 */
@Config
class DualClaw(hwMap: HardwareMap, startingPosition: Double = clawClosedPosition) : SubsystemBase() {
    private val servo = SimpleServo(hwMap, clawServoName, 0.0, 360.0)

    private enum class DualClawState {
        CLOSED,
        PARTIAL,
        FULL
    }

    private var clawState: DualClawState = DualClawState.CLOSED

    private var setpoint: Double = 0.0
        set(position) {
            field = clip(position, 0.0, 1.0)

            Log.i("Claw desired position", setpoint.toString())
        }


    class ClawPartial(private val claw: DualClaw) : Action {
        override fun run(p: TelemetryPacket): Boolean {
            claw.setPosition(clawPartiallyOpenedPosition)
            return false
        }
    }

    class ClawOpen(private val claw: DualClaw) : Action {
        override fun run(p: TelemetryPacket): Boolean {
            claw.setPosition(clawOpenedPosition)
            return false
        }
    }

    class ClawClose(private val claw: DualClaw) : Action {
        override fun run(p: TelemetryPacket): Boolean {
            claw.setPosition(clawClosedPosition)
            return false
        }
    }


    // Actions
    fun clawOpen(): Action {
        return ClawOpen(this)
    }
    fun clawPartial(): Action {
        return ClawPartial(this)
    }
    fun clawClose(): Action {
        return ClawClose(this)
    }

    init {
        setpoint = startingPosition
        servo.position = startingPosition
        clawState = DualClawState.CLOSED
    }

    override fun periodic() {
        Log.v("Claw estimated angle", getPositionEstimate().toString())
        servo.position = setpoint
    }

    /**
     * Sets position of the servo to a value between 0 and 1 (numbers outside of this range will be clipped).
     */
    fun setPosition(pos: Double) {
        servo.position = pos
    }

    /**
     * Opens the claw to release position (only meant to be used when depositing into the bucket).
     */
    fun release() {
        setpoint = clawReleasePosition
        clawState = DualClawState.FULL
    }

    /**
     * Open claw fully to pick up the thing
     */
    fun open() {
        setpoint = clawOpenedPosition
        clawState = DualClawState.FULL
    }

    /**
     * Open claw partially to pick up the thing
     */
    fun partiallyOpen() {
        setpoint = clawPartiallyOpenedPosition
        clawState = DualClawState.PARTIAL
    }

    /**
     * Close claw
     */
    fun close() {
        setpoint = clawClosedPosition
        clawState = DualClawState.CLOSED
    }

    /**
     * Increment open by 1
     */
    fun incrementOpen() {
        if (clawState == DualClawState.CLOSED) {
            setpoint = clawPartiallyOpenedPosition
            clawState = DualClawState.PARTIAL
        } else {
            setpoint = clawOpenedPosition
            clawState = DualClawState.FULL
        }
    }

    /**
     * Increment closed by 1
     */
    fun incrementClosed() {
        if (clawState == DualClawState.FULL) {
            setpoint = clawPartiallyOpenedPosition
            clawState = DualClawState.PARTIAL
        } else {
            setpoint = clawClosedPosition
            clawState = DualClawState.CLOSED
        }
    }

    /**
     * Confirms if claw is at target.
     */
    fun atTarget() : Boolean {
        //return timer.seconds() > motionProfile.duration
        return true
    }

    private fun getPositionEstimate() : Double {
        return servo.position
    }

    /**
     * Check if there is a cone based on color sensor.
     * @return [Boolean] of state.
     */
//    private val hsvValues = FloatArray(3)
//    fun isConeInside(): Boolean { //TODO either get motion profiling working or replace with non motion profiled approximation eg proportional to claw travel distance
//        val colors = colorSensor.normalizedColors
//        Color.colorToHSV(colors.toColor(), hsvValues)
//        Log.i("Color Sensor Values", hsvValues.toString())
//        return hsvValues[2] >= valueThreshold
//    }

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
        canvas.fillCircle(x, y, 2.0) //TODO measure claw radius
    }

    companion object {
        const val clawServoName = "claw"
        const val colorSensorName = "color"

        @JvmField
        var clawClosedPosition = 0.34
        @JvmField
        var clawPartiallyOpenedPosition = 0.45
        @JvmField
        var clawOpenedPosition = 0.69
        @JvmField
        var clawReleasePosition = 0.69


        @JvmField
        var clawMaxVel = 5.0
        @JvmField
        var clawMaxAccel = 5.0

        @JvmField
        var colorGain = 20.0
        @JvmField
        var valueThreshold = 0.15
    }

}