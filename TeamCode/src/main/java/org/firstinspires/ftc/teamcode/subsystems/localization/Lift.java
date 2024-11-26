package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.util.PIDFController;
import android.util.Log;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.filters.kalmanFilter.*;
import org.firstinspires.ftc.teamcode.util.PIDCoefficients;
import org.firstinspires.ftc.teamcode.util.ArrayToColumnMatrix;

import java.lang.Math;
import java.util.Arrays;

@Config
public class Lift extends SubsystemBase {

    private Motor leftMotor;
    private Motor rightMotor;
    private MotorGroup motorGroup;
    private TouchSensor limit;
    private VoltageSensor voltageSensor;
    private PIDFController controller;
    private TimeProfile motionProfile;
    private KalmanFilter filter;
    private double[] state = {0.0, 0.0, 0.0};
    private ElapsedTime profileTimer = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime loopTimer = new ElapsedTime();
    private double displacement;
    private double setpoint = 0.0;
    private boolean isRetracted = true;
    private LiftPosition lastPosition = LiftPosition.LOW;
    private boolean checkLimit = false;
    private double[] desiredState = new double[3];

    public enum LiftPosition {
        LOW(lowHeight),
        MIDDLE(midHeight),
        HIGH(highHeight);

        public final double height;

        LiftPosition(double height) {
            this.height = height;
        }
    }

    public Lift(HardwareMap hwMap, VoltageSensor voltageSensor) {
        this.voltageSensor = voltageSensor;

        leftMotor = new Motor(hwMap, leftLiftName);
        rightMotor = new Motor(hwMap, rightLiftName);
        motorGroup = new MotorGroup(leftMotor, rightMotor);

        limit = hwMap.get(TouchSensor.class, magnetLimitName);

        controller = new PIDFController(liftCoeffs, liftKStatic, liftKV, liftKA, (position, velocity) -> gravityFeedforward);

        SimpleMatrix H = new SimpleMatrix(new double[][]{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}});
        SimpleMatrix R = new SimpleMatrix(new double[][]{{0.00001, 0.0}, {0.0, 0.001}});
        LinearMeasurementModel measurementModel = new LinearMeasurementModel(H, R);
        ConstantAccelerationProcessModel processModel = new ConstantAccelerationProcessModel();

        SimpleMatrix initialState = new SimpleMatrix(new double[][]{{0.0, 0.0, 0.0}}).transpose();
        SimpleMatrix initialCovariance = new SimpleMatrix(3, 3);

        filter = new KalmanFilter(processModel, measurementModel, initialState, initialCovariance);

        resetEncoders();

        motorGroup.setDistancePerPulse(liftDPP);
        motorGroup.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        setpoint = 0.5;
        Log.i("Lift init setpoint", Double.toString(setpoint));
    }

    @Override
    public void periodic() {
        updateController();
        desiredState = getGlobalDesiredState();
        controller.setTargetPosition(desiredState[0]);
        controller.setTargetVelocity(desiredState[1]);
        controller.setTargetAcceleration(desiredState[2]);

        checkEncoder();
        double[] u = new double[desiredState.length];
        for (int i = 0; i < desiredState.length; i++) {
            u[i] = desiredState[i] - state[i];
        }
        updateFilter(ArrayToColumnMatrix.convert(u));
        setPower(controller.update(getRawExtensionLength(), getRawVelocity()));

        loopTimer.reset();
    }

    public void tuningPeriodic() {
        desiredState = getGlobalDesiredState();
        controller.setTargetPosition(desiredState[0]);
        controller.setTargetVelocity(desiredState[1]);
        controller.setTargetAcceleration(desiredState[2]);

        checkEncoder();
        double power = controller.update(getRawExtensionLength(), getRawVelocity());
        Log.i("lift power", Double.toString(power));
        setPower(power);
        timer.reset();
    }

    private void updateFilter(SimpleMatrix u) {
        double dt = timer.time();
        filter.predict(u, dt);
        double[] z = {getRawExtensionLength(), getRawVelocity()};
        filter.update(ArrayToColumnMatrix.convert(z));
        state = filter.getStateEstimate().getDDRM().data;
        timer.reset();
    }

    private void checkEncoder() {
        if (checkLimit && getExtensionLength() <= withinSwitchRange) {
            if (limit.isPressed()) {
                resetEncoders();
                checkLimit = false;
                Log.i("Limit Switch", "Resetting");
            }
        }
    }

    private void resetEncoders() {
        leftMotor.resetEncoder();
        rightMotor.resetEncoder();
    }

    private void updateController() {
        controller.setPID(liftCoeffs);
        controller.setkStatic(liftKStatic);
        controller.setkV(liftKV);
        controller.setkA(liftKA);
        controller.setkF((position, velocity) -> gravityFeedforward);
    }

    private double[] getGlobalDesiredState() {
        double[] displacementDesiredState = motionProfile.get(profileTimer.seconds()).values();
        double[] globalDesiredState = new double[3];
        globalDesiredState[0] = (setpoint - displacement) + (Math.signum(displacement) * displacementDesiredState[0]);
        globalDesiredState[1] = displacementDesiredState[1] * Math.signum(displacement);
        globalDesiredState[2] = displacementDesiredState[2] * Math.signum(displacement);
        return globalDesiredState;
    }

    public void setHeight(double height) {
        double targetLength = (height - liftHeightOffset) / Math.sin(Math.toRadians(liftAngle));
        setpoint = targetLength;
        checkLimit = false;
        isRetracted = false;
    }

    public void setHeight(LiftPosition pos) {
        setHeight(pos.height);
        lastPosition = pos;
    }

    public void retract() {
        setpoint = 0.0;
        checkLimit = true;
        isRetracted = true;
    }

    public void setPower(double power) {
        if ((desiredState[1] > 0 && getRawExtensionLength() > 30.0) || (desiredState[1] < 0 && getRawExtensionLength() < 0.5)) {
            motorGroup.set(0.0);
        } else {
            motorGroup.set(power * 12.0 / voltageSensor.getVoltage());
        }
    }

    public double getExtensionLength() {
        return state[0];
    }

    public double getVelocity() {
        return state[1];
    }

    public double getAcceleration() {
        return state[2];
    }

    public double getRawExtensionLength() {
        return Arrays.stream(motorGroup.getPositions()).average().orElse(0.0);
    }

    public double getRawVelocity() {
        return Arrays.stream(motorGroup.getVelocities()).average().orElse(0.0);
    }

    public boolean atTarget() {
        return Math.abs(getRawExtensionLength() - setpoint) < liftTargetErrorTolerance;
    }

    public double timeFromTarget() {
        return motionProfile.duration - profileTimer.seconds();
    }

    public void fetchTelemetry(TelemetryPacket packet) {
        packet.put("Battery adjustment factor", 12.0 / voltageSensor.getVoltage());
        packet.put("Lift raw length", getRawExtensionLength());
        packet.put("Lift raw velocity", getRawVelocity());
        packet.put("Lift estimated length", getExtensionLength());
        packet.put("Lift estimated velocity", getVelocity());
        packet.put("Lift estimated acceleration", getAcceleration());
        packet.put("Target position", desiredState[0]);
        packet.put("Position error", desiredState[0] - getRawExtensionLength());
        packet.put("Target velocity", desiredState[1]);
        packet.put("Velocity Error", desiredState[1] - getRawVelocity());
    }

    public static final String leftLiftName = "liftLeft";
    public static final String rightLiftName = "liftRight";
    public static final String magnetLimitName = "magnet";
    public static final double liftHeightOffset = 7.5;
    public static final double liftMaxExtension = 27.0;
    public static final double liftDPP = 0.00673;
    public static final double liftOffsetDistanceFromCenter = 0.0;
    public static final double liftAngle = 36.98;
    public static final double lowHeight = 16.0;
    public static final double midHeight = 20.0;
    public static final double highHeight = 27.0;
    public static final double liftMaxVel = 15.0;
    public static final double liftMaxAccel = 40.0;
    public static final double liftKStatic = 0.05;
    public static final double liftKV = 0.053;
    public static final double liftKA = 0.0035;
    public static final double gravityFeedforward = 0.045;
    public static final PIDCoefficients liftCoeffs = new PIDCoefficients(0.2, 0.0, 0.007);
    public static final double liftTargetErrorTolerance = 0.7;
    public static final double withinSwitchRange = 0.5;
}
