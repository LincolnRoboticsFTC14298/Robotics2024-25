package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;
import android.util.Log;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.Range;

@Config
public class Claw extends SubsystemBase {

    private SimpleServo servo;
    private double setpoint;

    public Claw(HardwareMap hwMap, double startingPosition) {
        servo = new SimpleServo(hwMap, clawServoName, 0.0, 360.0);
        setpoint = Range.clip(startingPosition, 0.0, 1.0);
        servo.setPosition(startingPosition);
    }

    @Override
    public void periodic() {
        Log.v("Claw estimated angle", Double.toString(getPositionEstimate()));
        servo.setPosition(setpoint);
    }

    public void open() {
        setpoint = clawOpenedPosition;
    }

    public void partiallyOpen() {
        setpoint = clawPartiallyOpenedPosition;
    }

    public void close() {
        setpoint = clawClosedPosition;
    }

    public boolean atTarget() {
        return true;
    }

    private double getPositionEstimate() {
        return servo.getPosition();
    }

    public void fetchTelemetry(TelemetryPacket packet) {
        packet.put("Position Estimate", getPositionEstimate());
        packet.put("Desired position", setpoint);
    }

    public void drawClaw(Canvas canvas, Vector2d clawOffset, Pose2d pose) {
        canvas.setStroke("#8CA231");
        Vector2d position = pose.getPosition().plus(pose.getHeading().inverse().times(clawOffset));
        canvas.fillCircle(position.getX(), position.getY(), 2.0);
    }

    public static final String clawServoName = "claw";
    public static final double clawClosedPosition = 0.05;
    public static final double clawOpenedPosition = 0.15;
    public static final double clawPartiallyOpenedPosition = 0.80;
    public static final double clawMaxVel = 5.0;
    public static final double clawMaxAccel = 5.0;
    public static final double colorGain = 20.0;
    public static final double valueThreshold = 0.15;
}
