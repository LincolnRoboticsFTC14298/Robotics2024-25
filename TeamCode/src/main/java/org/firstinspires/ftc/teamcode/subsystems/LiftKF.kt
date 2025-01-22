package org.firstinspires.ftc.teamcode.subsystems

import org.firstinspires.ftc.teamcode.util.PIDFController
import android.util.Log
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.*
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.TouchSensor
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.ejml.simple.SimpleMatrix
import org.firstinspires.ftc.teamcode.filters.kalmanFilter.*
import org.firstinspires.ftc.teamcode.util.PIDCoefficients
import org.firstinspires.ftc.teamcode.util.arrayToColumnMatrix
import java.lang.Math.toRadians
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sign
import kotlin.math.sin

/**
 * Lift consists of two multistage slides powered by a motor which pulls a string.
 * @param hwMap        HardwareMap.
 */
@Config
class LiftKF(hwMap: HardwareMap, private val voltageSensor: VoltageSensor) : SubsystemBase() {

    /**
     * Avoid using the individual motors, it's best to use the group.
     * @see <a href="https://docs.ftclib.org/ftclib/features/hardware/motors">FTCLib Docs: Motors</a>
     */
    private val leftMotor  = Motor(hwMap, leftLiftName)
    private val rightMotor = Motor(hwMap, rightLiftName)
    private val motorGroup = MotorGroup(leftMotor, rightMotor)

    private val limit = hwMap.get(TouchSensor::class.java, magnetLimitName)

    private var controller = PIDFController(liftCoeffs, liftKStatic, liftKV, liftKA, {_, _ -> gravityFeedforward})
    private lateinit var motionProfile: TimeProfile

    private var filter: KalmanFilter
    private var state: DoubleArray = doubleArrayOf(0.0, 0.0, 0.0)

    private val profileTimer = ElapsedTime()
    private val timer = ElapsedTime()
    private val loopTimer = ElapsedTime()


    private var displacement: Double = 0.0

    enum class LiftPosition(val height: Double) {
        LOW(lowHeight),
        MIDDLE(midHeight),
        HIGH(highHeight)
    }

    var lastPosition: LiftPosition = LiftPosition.LOW

    /**
     * @return Target height off the ground in in.
     */
    var setpoint: Double = 0.0
        /**
         * Sets the target extension length and constructs an optimal motion profile.
         * @param height        Target length in inches.
         */
        set(length) {
            Log.i("Lift setpoint attempt", length.toString())
            field = Range.clip(length, 0.0, liftMaxExtension)
            displacement = field - getRawExtensionLength() //TODO also change to setpoint here
            if (displacement != 0.0) { //TODO change getExtensionLength() to setpoint to get displacement between previous and current values
                profileTimer.reset()
                motionProfile = TimeProfile(constantProfile(abs(displacement), 0.0, liftMaxVel, -liftMaxAccel, liftMaxAccel).baseProfile)
            }
        }

    init {

        val processModel = ConstantAccelerationProcessModel()

        val H = SimpleMatrix(arrayOf(doubleArrayOf(1.0, 0.0, 0.0), doubleArrayOf(0.0, 1.0, 0.0)))
        val R = SimpleMatrix(arrayOf(doubleArrayOf(0.00001, 0.0), doubleArrayOf(0.0, 0.001)))
        val measurementModel = LinearMeasurementModel(H, R)

        // Retracted and stationary
        val initialState = SimpleMatrix(arrayOf(doubleArrayOf(0.0, 0.0, 0.0))).transpose()
        // Completely sure that it is retracted and stationary
        val initialCovariance = SimpleMatrix(3, 3)
        filter = KalmanFilter(processModel, measurementModel, initialState, initialCovariance)

        resetEncoders()

        motorGroup.setDistancePerPulse(liftDPP)
        motorGroup.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT)

        retract()
    }

    var desiredState: DoubleArray = DoubleArray(3)
    override fun periodic() {
        // TODO: Maybe only set power if it has actually changed!! Do this through thresholding
        //  integrating current power with desired power. Write wrappers for automatic voltage
        //  compensation.
        //  Naive optimization would only write when motion profiling is active; heavily trusts ff

        desiredState = getGlobalDesiredState()

        controller.apply {
            targetPosition = desiredState[0]
            targetVelocity = desiredState[1]
            targetAcceleration = desiredState[2]
        }

        checkEncoder()

        // Get current state estimates using kalman filter//
        val u = desiredState.zip(state).map { it.first - it.second }.toDoubleArray()
        updateFilter(arrayToColumnMatrix(u))

        setPower(controller.update(getExtensionLength(), getVelocity()))

        timer.reset()

        Log.v("Loop duration", loopTimer.seconds().toString())
        loopTimer.reset()

        Log.v("getRawExtensionLength", getRawExtensionLength().toString())
        Log.v("getExtensionLength", getExtensionLength().toString())
    }


    fun tuningPeriodic() {

        desiredState = getGlobalDesiredState()
        controller.apply {
            targetPosition = desiredState[0]
            targetVelocity = desiredState[1]
            targetAcceleration = desiredState[2]
        }

        checkEncoder()

        val power = controller.update(getRawExtensionLength(), getRawVelocity())
        Log.i("lift power", power.toString())
        setPower(power)

        timer.reset()
    }

    fun updateFilter(u: SimpleMatrix?) {
        val dt = timer.time()
        filter.predict(u, dt)

        val z = doubleArrayOf(getRawExtensionLength(), getRawVelocity())
        Log.v("Raw state", z.toString())
        filter.update(arrayToColumnMatrix(z))

        state = filter.stateEstimate.ddrm.data
        Log.v("Estimated state", state.toString())

        timer.reset()
    }

    /**
     * Resets encoder position if necessary
     */
    var checkLimit = false
    fun checkEncoder() {
        if (checkLimit && getExtensionLength() <= withinSwitchRange) {
            if (limit.isPressed) {
                resetEncoders()
                checkLimit = false // Only need to check limit switch once
                Log.i("Limit Switch", "Resetting")
            }
        }
    }

    fun resetEncoders() {
        leftMotor.resetEncoder()
        rightMotor.resetEncoder()
    }

    fun updateController() {
        controller.pid = liftCoeffs
        controller.kStatic = liftKStatic
        controller.kV = liftKV
        controller.kA = liftKA
        controller.kF = {_, _ -> gravityFeedforward}
    }

    fun getGlobalDesiredState(): DoubleArray {
        val displacementDesiredState = motionProfile[profileTimer.seconds()].values().toDoubleArray()
        Log.v("disp desired state", displacementDesiredState.contentToString())
        val globalDesiredState = DoubleArray(3)
        globalDesiredState[0] = (setpoint - displacement) + (sign(displacement) * displacementDesiredState[0])
        globalDesiredState[1] = displacementDesiredState[1] * sign(displacement)
        globalDesiredState[2] = displacementDesiredState[2] * sign(displacement)
        Log.v("global desired state", globalDesiredState.contentToString())
        return globalDesiredState
    }

    fun setHeight(height: Double) {
        val targetLength = (height - liftHeightOffset) / sin(toRadians(liftAngle)) // It will be extending upwards so no need to check
        setpoint = targetLength
        checkLimit = false
    }
    /**
     * Sets the target height of the lift and constructs an optimal motion profile for it.
     * @param pole          Based on [PoleType] heights.
     */

    fun setHeight(pos: LiftPosition) {
        setHeight(pos.height)
        lastPosition = pos
    }

    fun setHeightLastPos() {
        setHeight(lastPosition)
    }

    /**
     * Retracts the lift to the starting height.
     */
    fun retract() {
        setpoint = 0.01
        checkLimit = true
    }

    /**
     * Set power of lift.
     * @param power         Percentage of the maximum speed of the lift.
     */
    fun setPower(power: Double) {
        if ((desiredState[1] > 0 && getRawExtensionLength() > 30.0) || (desiredState[1] < 0 && getRawExtensionLength() < 0.5)) {
            motorGroup.set(0.0)
        } else {
            motorGroup.set(power * 12.0 / voltageSensor.voltage)
        }

    }

    /**
     * Returns the position of where the passthrough is attached in tangent space.
     */
    fun getRelativePosition(): Pose2d {
        return Pose2d(getExtensionLength() * cos(toRadians(liftAngle)) + liftOffsetDistanceFromCenter, 0.0, 0.0)
    }

    fun getFutureRelativePosition(): Pose2d {
        return Pose2d(setpoint * cos(toRadians(liftAngle)) + liftOffsetDistanceFromCenter, 0.0, 0.0)
    }

    /**
     * @return Distance the lift has extended relative to retracted state in in.
     */
    fun getExtensionLength(): Double {
        return state[0]
    }

    /**
     * @return Velocity of the lift in in / s.
     */
    fun getVelocity(): Double {
        return state[1]
    }

    /**
     * @return Acceleration of the lift in in / s2.
     */
    fun getAcceleration(): Double {
        return state[2]
    }

    /**
     * @return Raw lift position has extended relative to retracted state in in.
     */
    fun getRawExtensionLength(): Double {
        return motorGroup.positions.average()
    }

    /**
     * @return Raw velocity of the lift in in / s.
     */
    fun getRawVelocity(): Double {
        return motorGroup.velocities.average()
    }


    /**
     * @return True if the controller has reached the target with some tolerance.
     */
    fun atTarget(): Boolean {
        return abs(getRawExtensionLength() - setpoint) < liftTargetErrorTolerance
    }


    /**
     * @return Time remaining from reaching the target.
     */
    fun timeFromTarget(): Double {
        return motionProfile.duration - profileTimer.seconds()
    }

    /**
     * For debugging/tuning purposes
     */
    fun fetchTelemetry(packet: TelemetryPacket) {
        packet.put("Battery adjustment factor", 12.0 / voltageSensor.voltage)
        packet.put("Lift raw length", getRawExtensionLength())
        packet.put("Lift raw velocity", getRawVelocity())
        //packet.put("Lift raw acceleration", motorGroup.encoder.acceleration * liftDPP)
        packet.put("Lift estimated length", getExtensionLength())
        packet.put("Lift estimated velocity", getVelocity())
        packet.put("Lift estimated acceleration", getAcceleration())
        packet.put("Target position", desiredState[0])
        packet.put("Position error", desiredState[0] - getRawExtensionLength())
        packet.put("Target velocity", desiredState[1])
        packet.put("Velocity Error", desiredState[1] - getRawVelocity())

        Log.v("getRawExtensionLength", getRawExtensionLength().toString())
        Log.v("getExtensionLength", getExtensionLength().toString())
    }

    companion object {
        const val leftLiftName = "liftLeft"
        const val rightLiftName = "liftRight"
        const val magnetLimitName = "magnet"

        const val liftHeightOffset = 7.5 // in The raw height of zero is off the ground

        const val liftMaxExtension = 27.0 // in Max allowable extension height

        const val liftDPP = 0.00673

        const val liftOffsetDistanceFromCenter = 0.0 // ignore

        const val liftAngle = 36.98 // deg

        @JvmField
        var lowHeight = 14.0
        @JvmField
        var midHeight = 20.0
        @JvmField
        var highHeight = 25.0

        @JvmField
        var liftMaxVel = 15.0 // in / s  // TODO: Find max values
        @JvmField
        var liftMaxAccel = 40.0 // in / s2

        @JvmField
        var liftKStatic = 0.1397
        @JvmField
        var liftKV = 0.057
        @JvmField
        var liftKA = 0.0035
        @JvmField
        var gravityFeedforward = 0.0435


        @JvmField
        var liftCoeffs = PIDCoefficients(0.2, 0.0, 0.0) // TODO: Calculate from kV and kA

        val liftTargetErrorTolerance = 0.7 // in

        const val withinSwitchRange = 0.5 // in from the bottom to check magnet switch for reset
    }

}