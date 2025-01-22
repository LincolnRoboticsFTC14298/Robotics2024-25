package org.firstinspires.ftc.teamcode.vision.modules

//import android.util.Log
import com.acmerobotics.roadrunner.Vector2d
import org.firstinspires.ftc.teamcode.FieldConfig
import org.firstinspires.ftc.teamcode.subsystems.Vision
//import org.firstinspires.ftc.teamcode.util.LogFiles.log
import org.firstinspires.ftc.teamcode.util.epsilonEquals
import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.MatOfPoint2f
import org.opencv.core.Point
import org.opencv.imgproc.Imgproc.*
import java.lang.Math.toDegrees
import kotlin.math.min
import kotlin.math.tan
import kotlin.math.cos
import kotlin.math.sin


/**
 * Returns contours that pass the scorers by thresholding a weighted sum.
 * @param cameraHeight must be in same units as returned distance
 */
class BlobResults(
        private val maskModule: AbstractPipelineModule<Mat>,
        private val camera: Vision.Companion.CameraData,
) : AbstractPipelineModule<BlobResults.AnalysisResult>() {

    data class AnalysisResult(val pixelPoint:Point, val yaw: Double, val pitch: Double) {

    }

    init {
        addParentModules(maskModule)
    }

    override fun processFrameForCache(rawInput: Mat): AnalysisResult {
        val mask = maskModule.processFrame(rawInput)

        // Find the coordinates of non-zero pixels
        val nonZeroCoordinates = ArrayList<Point>()
        for (y in 0 until mask.rows()) {
            for (x in 0 until mask.cols()) {
                if (mask.get(y, x)[0] > 0) {
                    nonZeroCoordinates.add(Point(x.toDouble(), y.toDouble()))
                }
            }
        }

        // Calculate the average coordinates
        var averageLocation = nonZeroCoordinates.fold(Point(0.0, 0.0)) { acc, point ->
            Point(acc.x + point.x, acc.y + point.y)
        }
        averageLocation = Point(averageLocation.x / (nonZeroCoordinates.size.toDouble() + 0.01), averageLocation.y / (nonZeroCoordinates.size.toDouble() + 0.01))

        val w = rawInput.size().width
        val h = rawInput.size().height
        val Ax = (averageLocation.x - w/2.0) / (w/2.0)
        val Ay = (h/2.0 - averageLocation.y) / (h/2.0)

        // calculate angle and distance
        val pitch = Ay * camera.FOVY / 2.0
        val yaw = Ax * camera.FOVX / 2.0

        return AnalysisResult(averageLocation, yaw, pitch)
    }


}