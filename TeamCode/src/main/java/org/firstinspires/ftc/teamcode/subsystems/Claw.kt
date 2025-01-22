package org.firstinspires.ftc.teamcode.subsystems

import android.graphics.Color
import android.util.Log
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.TimeProfile
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.constantProfile
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.SimpleServo
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range.clip
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.FieldConfig

/**
 * Claw subsystem consists of a servo, potentiometer and a color sensor.
 * @param hwMap HardwareMap.
 */
@Config
class Claw(hwMap: HardwareMap, startingPosition: Double = clawClosedPosition) : SubsystemBase() {

    private val servo = SimpleServo(hwMap, clawServoName, 0.0, 360.0)

    private var setpoint: Double = 0.0
        set(position) {
            field = clip(position, 0.0, 1.0)
            Log.i("Claw desired position", setpoint.toString())
        }

    init {
        setpoint = startingPosition
        servo.position = startingPosition
    }

    override fun periodic() {
        Log.v("Claw estimated angle", getPositionEstimate().toString())
        servo.position = setpoint
    }

    /**
     * Open claw to pick up the thing. TODO make better name for "thing" yk yk
     */
    fun open() {
        setpoint = clawOpenedPosition
    }

    /**
     * Close claw to grab the thing.
     */
    fun close() {
        setpoint = clawClosedPosition
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
        var clawClosedPosition = 0.10
        @JvmField
        var clawOpenedPosition = 0.15


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