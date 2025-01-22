package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.subsystems.localization.StartingPoseStorage
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline
import org.firstinspires.ftc.teamcode.vision.SpikeDetectionPipeline
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.MatOfDouble
import org.openftc.apriltag.AprilTagDetection
import org.openftc.apriltag.AprilTagPose
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvPipeline
import kotlin.math.cos
import kotlin.math.sin
import org.firstinspires.ftc.teamcode.FieldConfig.AprilTagResult
import org.opencv.core.Point

/*
TODO Research veiwportContainerIds
TODO Create different pipelines for apriltag, *pixels, spike mark
TODO add april tag data to fieldConfig/cameraConfig
TODO search vision subsystem for additions and changes to make
TODO add method for returning Pose2D from aprilTagID
TODO not urgent: python/java script train convexity and aspect ratio values (mean and variance) might already exist in test pipeline
TODO write a pipline for detecting a cone of a specific color, write a tuning opmode for that pipline with editable filter parameters via the dashboard like in the Mecanum and Vision subsystems for ease of tuning
 */


/**
 * Manages all the pipelines and cameras.
 */
@Config
class Vision(
    hwMap: HardwareMap,
    var startingPipeline: FrontPipeline = FrontPipeline.RED_SPIKE_PIPELINE,
    private val telemetry: Telemetry? = null
) : SubsystemBase() {

    val cameraMonitorViewId = hwMap.appContext.resources.getIdentifier(
        "cameraMonitorViewId",
        "id",
        hwMap.appContext.packageName
    )

    var viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY)
    val webCam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName::class.java, "Webcam 1"), viewportContainerIds[0])
    //val phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, viewportContainerIds[1])

    private val dashboard = FtcDashboard.getInstance()


    enum class FrontPipeline(var pipeline: OpenCvPipeline) {
        APRIL_TAG(AprilTagDetectionPipeline(CameraData.LOGITECH_C920)),
        RED_SPIKE_PIPELINE(SpikeDetectionPipeline(SpikeDetectionPipeline.DisplayMode.MARKER, CameraData.LOGITECH_C920, true, telemetry)),
        BLUE_SPIKE_PIPELINE(SpikeDetectionPipeline(SpikeDetectionPipeline.DisplayMode.MARKER, CameraData.LOGITECH_C920, false, telemetry))
    }

    //val phoneCamPipeline = GeneralPipeline(GeneralPipeline.DisplayMode.ALL_CONTOURS, CameraData.PHONECAM, telemetry)

    init {
        name = "Vision Subsystem"

        if (startingPipeline == FrontPipeline.RED_SPIKE_PIPELINE || startingPipeline == FrontPipeline.BLUE_SPIKE_PIPELINE) { //also hacky but whatever
            startingPipeline = if (StartingPoseStorage.startingPose.isRedAlliance()) {
                FrontPipeline.RED_SPIKE_PIPELINE
            } else {
                FrontPipeline.BLUE_SPIKE_PIPELINE
            }
        }

        (startingPipeline.pipeline as SpikeDetectionPipeline).telemetry = telemetry



        webCam.openCameraDeviceAsync(object : AsyncCameraOpenListener {
            override fun onOpened() {
                webCam.setPipeline(startingPipeline.pipeline)
                webCam.showFpsMeterOnViewport(true)
                //webCam.setViewportRenderer(OpenCvCamera.ViewportRenderer.NATIVE_VIEW)
                startStreamingFrontCamera()

            }

            override fun onError(errorCode: Int) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        })
    }

    /**
     * Starts streaming the front camera.
     */
    fun startStreamingFrontCamera() {
        webCam.startStreaming(160, 90, OpenCvCameraRotation.UPRIGHT ) // TODO Check
        dashboard.startCameraStream(webCam, 10.0)
    }

    /**
     * Stops streaming the front camera.
     */
    fun stopStreamingFrontCamera() {
        webCam.stopStreaming()
        dashboard.stopCameraStream()
        lastSpikeFrame = 0
    }

    /**
     * Sets the front pipeline.
     * @param pipeline     From the [FrontPipeline] pipeline options.
     */
    fun setFrontPipeline(pipeline: FrontPipeline) {
        webCam.setPipeline(pipeline.pipeline)
    }

    var numFramesWithoutDetection = 0

    private val DECIMATION_HIGH = 3f
    private val DECIMATION_LOW = 2f
    private val THRESHOLD_HIGH_DECIMATION_RANGE_FEET = 3.0f
    private val THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4



    fun updateAprilTag() : ArrayList<AprilTagDetection>? {
        // Calling getDetectionsUpdate() will only return an object if there was a new frame
        // processed since the last time we called it. Otherwise, it will return null. This
        // enables us to only run logic when there has been a new frame, as opposed to the
        // getLatestDetections() method which will always return an object.
        val aprilTagDetectionPipeline = (FrontPipeline.APRIL_TAG.pipeline as AprilTagDetectionPipeline)
        val detections: ArrayList<AprilTagDetection>? = aprilTagDetectionPipeline.getDetectionsUpdate()

        // If there's been a new frame...
        if (detections != null) {

            // If we don't see any tags
            if (detections.size == 0) {
                numFramesWithoutDetection++

                // If we haven't seen a tag for a few frames, lower the decimation
                // so we can hopefully pick one up if we're e.g. far back
                if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                    aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW)
                }
            } else {
                numFramesWithoutDetection = 0

                // If the target is within 1 meter, turn on high decimation to
                // increase the frame rate
                if (detections[0].pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_FEET) {
                    aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH)
                }

                return detections//AprilTagResult.find(detections.minBy{ it.pose.z }.id)
            }
        }

        return null
    }

    fun getLeftAprilTag() : AprilTagPose? {
        return (FrontPipeline.APRIL_TAG.pipeline as AprilTagDetectionPipeline).poseFromId(if (StartingPoseStorage.startingPose.isRedAlliance()) {
            AprilTagResult.BACKDROP_LEFT_RED.id
        } else {
            AprilTagResult.BACKDROP_LEFT_BLUE.id
        })
    }

    fun getCenterAprilTag() : AprilTagPose? {
        return (FrontPipeline.APRIL_TAG.pipeline as AprilTagDetectionPipeline).poseFromId(if (StartingPoseStorage.startingPose.isRedAlliance()) {
            AprilTagResult.BACKDROP_MIDDLE_RED.id
        } else {
            AprilTagResult.BACKDROP_MIDDLE_BLUE.id
        })
    }

    fun getRightAprilTag() : AprilTagPose? {
        return (FrontPipeline.APRIL_TAG.pipeline as AprilTagDetectionPipeline).poseFromId(if (StartingPoseStorage.startingPose.isRedAlliance()) {
            AprilTagResult.BACKDROP_RIGHT_RED.id
        } else {
            AprilTagResult.BACKDROP_RIGHT_BLUE.id
        })
    }

    data class ObservationResult(val angle: Double, val distance: Double) {

        companion object {

            fun fromVector(vector: Vector2d) : ObservationResult {
               return ObservationResult(vector.angleCast().log(), vector.norm())
            }
        }
        fun toVector() = Vector2d(distance * cos(angle), distance * sin(angle))

        fun distance(other: ObservationResult) = (toVector() - other.toVector()).norm()

        fun sqrDistance(other: ObservationResult) = (toVector() - other.toVector()).sqrNorm()

        operator fun plus(vector: Vector2d) = fromVector(vector + toVector())

        override fun toString() = String.format("Angle: %.1f, Distance: %.2f", Math.toDegrees(angle), distance)
    }

    /**
     * @return List of pixel info for landmarks from pipeline
     * TODO: Include tall stacks not next to poles
     */

    fun getSpikeMarkDetection(): Point { //hacky but ehh itll work for comp - change back later tho - (3 days or so later) hah you thought THAT was bad??? well boy do I have news for you.

        val spikeResults =
            if (StartingPoseStorage.startingPose.isRedAlliance()) (FrontPipeline.RED_SPIKE_PIPELINE.pipeline as SpikeDetectionPipeline).spikeResults else (FrontPipeline.BLUE_SPIKE_PIPELINE.pipeline as SpikeDetectionPipeline).spikeResults

        return Point(spikeResults.yaw, spikeResults.pitch)
    }



    //fun getSpikeInfo(): List<Vector2d> = getSpikeMarkDetections().map{ it - Vector2d(0.0, CameraData.LOGITECH_C920.pitch) }

    val leftSpikeCutoff = -0.35 //TODO verify these values
    val rightSpikeCutoff = 0.0

    enum class SpikeDirection() {
        LEFT(),
        CENTER(),
        RIGHT()
    }

    private var lastSpikeFrame = 0
    var spikeLocation = 0.0
    fun getSpikeMarkDirectionUpdate(): SpikeDirection? {
        val spike = getSpikeMarkDetection()

        if (spike.y<0.4 && spike.x>-0.4) {
            if (lastSpikeFrame != webCam.frameCount) {
                lastSpikeFrame = webCam.frameCount
                val xCoord = spike.x
                spikeLocation = xCoord

                return if (xCoord > rightSpikeCutoff) {
                    SpikeDirection.RIGHT
                } else if (xCoord > leftSpikeCutoff) {
                    SpikeDirection.CENTER
                } else {
                    SpikeDirection.LEFT
                }
            }
        }
        return null
    }

    private fun drawObservationResult(canvas: Canvas, observation: ObservationResult, pose: Pose2d, radius: Double, fill: Boolean = true) {
        val (x, y) = pose.position.plus(pose.heading.inverse().times(observation.toVector()))
        if (fill) canvas.fillCircle(x, y, radius)
        else canvas.strokeCircle(x, y, radius)
    }

    companion object {
        enum class CameraData(val pitch: Double, val height: Double, val relativePosition: Vector2d, val FOVX: Double, val FOVY: Double, val fx: Double, val fy: Double, val cx: Double, val cy: Double, val distCoeffs: MatOfDouble) {
            PHONECAM(0.0, 2.0, Vector2d(-5.0, 0.0),
                Math.toRadians(60.0),
                Math.toRadians(60.0), 0.0, 0.0, 0.0, 0.0, MatOfDouble(0.0, 0.0, 0.0, 0.0, 0.0)
            ),
            LOGITECH_C920(0.01, 5.44, Vector2d(4.5, 0.0),
                Math.toRadians(70.42),
                Math.toRadians(43.3), 477.73045982, 479.24207234, 311.48519892, 176.10784813, MatOfDouble(0.07622862, -0.41153656, -0.00089351, 0.00219123, 0.57699695)
            );

            fun getCameraMatrix(): Mat {
                val cameraMat = Mat(3, 3, CvType.CV_64FC1)
                val cameraData = doubleArrayOf(fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0)
                cameraMat.put(0, 0, *cameraData)
                return cameraMat
            }
        }
    }

}