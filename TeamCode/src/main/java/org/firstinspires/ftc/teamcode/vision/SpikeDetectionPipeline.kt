package org.firstinspires.ftc.teamcode.vision

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.Vision
import org.firstinspires.ftc.teamcode.vision.modulelib.InputModule
import org.firstinspires.ftc.teamcode.vision.modulelib.ModularPipeline
import org.firstinspires.ftc.teamcode.vision.modules.*
import org.firstinspires.ftc.teamcode.vision.modules.features.AspectRatio
import org.firstinspires.ftc.teamcode.vision.modules.features.Convexity
import org.firstinspires.ftc.teamcode.vision.modules.features.Extent
import org.firstinspires.ftc.teamcode.vision.modules.features.Solidity
import org.firstinspires.ftc.teamcode.vision.modules.scorers.DiffSquaredScorer
import org.firstinspires.ftc.teamcode.vision.modules.scorers.plus
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.opencv.imgproc.Imgproc.drawMarker


open class SpikeDetectionPipeline(
    private var displayMode: DisplayMode = DisplayMode.MARKER,
    camera: Vision.Companion.CameraData,
    isRedAlliance: Boolean,
    var telemetry: Telemetry?
) : ModularPipeline() {

    //private val camMat = Mat()
    //private val distCoeffs = Mat()

    enum class DisplayMode {
        RAW_CAMERA_INPUT,
        MARKER,
        RAW_SPIKE_MASK,
        DENOISED_SPIKE_MASK
    }

    // Modules //
    private val inputModule = InputModule()
    private val labColorSpace = ColorConverter(inputModule, Imgproc.COLOR_RGB2Lab)
    private val spikeMask: Filter = if(isRedAlliance){Filter(labColorSpace, Scalar(0.0, 145.0, 110.0), Scalar(255.0, 210.0, 190.0))} else {Filter(labColorSpace, Scalar(0.0, 58.0, 70.0), Scalar(255.0, 121.0, 121.0))}
    private val denoisedSpikeMask = Denoise(spikeMask, 15, 15, 3, 3)
//    private val rawSpikeContours = Contours(denoisedSpikeMask)
//
//OLD VALUES
//    // Spike Mark Scorer //
//    private val spikeConvexityScorer = DiffSquaredScorer(Convexity(), 0.988, 11.09)
//    private val spikeExtentScorer = DiffSquaredScorer(Extent(), 0.661, 0.036)
//    private val spikeSolidityScorer = DiffSquaredScorer(Solidity(), 0.909, 0.227)
//    private val spikeAspectRatioScorer = DiffSquaredScorer(AspectRatio(), 1.369, 0.23)
//    private val spikeContours = FilterContours(rawSpikeContours, 0.05, spikeConvexityScorer + spikeExtentScorer + spikeSolidityScorer + spikeAspectRatioScorer)

//NEW VALUES
//    // Spike Mark Scorer //
//    private val spikeConvexityScorer = DiffSquaredScorer(Convexity(), 0.97, 7.7)
//    private val spikeExtentScorer = DiffSquaredScorer(Extent(), 0.87, 8.3)
//    private val spikeSolidityScorer = DiffSquaredScorer(Solidity(), 0.97, 1.15)
//    private val spikeAspectRatioScorer = DiffSquaredScorer(AspectRatio(), 1.2, 9.4)
//    private val spikeContours = FilterContours(rawSpikeContours, 0.05, spikeConvexityScorer + spikeExtentScorer + spikeSolidityScorer + spikeAspectRatioScorer)


    
    // Results Modules //
    //private val spikeResultsModule = ContourResults(spikeContours, camera, poleDiameter, poleBaseHeight) //TODO MEASURE AND CHANGE OFFSETS
    private val spikeResultsModule = BlobResults(spikeMask, camera)

    // Data we care about and wish to access
    var spikeResults = BlobResults.AnalysisResult(Point(), 0.0, 0.0)

    init {
        addEndModules(spikeResultsModule)
    }

    override fun processFrameForCache(input: Mat) : Mat {

        // Get the data we want (yipee)
        spikeResults = spikeResultsModule.processFrame(input)


        // Telemetry for Testing //
        telemetry?.addData("displaymode", displayMode)

//        telemetry?.addLine("mean, variance")
//        telemetry?.addData("aspectRatio", spikeAspectRatioScorer.feature.mean().toString() + ", " + spikeAspectRatioScorer.feature.variance().toString())
//        telemetry?.addData("convexity", spikeConvexityScorer.feature.mean().toString() + ", " + spikeConvexityScorer.feature.variance().toString())
//        telemetry?.addData("extent", spikeExtentScorer.feature.mean().toString() + ", " + spikeExtentScorer.feature.variance().toString())
//        telemetry?.addData("solidity", spikeSolidityScorer.feature.mean().toString() + ", " + spikeSolidityScorer.feature.variance().toString())
//        telemetry?.addData("aspectRatio min, max", (spikeAspectRatioScorer.feature as AspectRatio).min().toString() + ", " + spikeAspectRatioScorer.feature.max().toString())
//        //telemetry.addData("areaResults", poleArea.areaResultsList().sorted().toString())

        telemetry?.addData("Average Point", spikeResults.pixelPoint.toString())
        telemetry?.addData("yaw, pitch", spikeResults.yaw.toString() + " ," + spikeResults.pitch.toString())


        telemetry?.update()


        // Display //
        return when (displayMode) {
            DisplayMode.RAW_CAMERA_INPUT -> input
            DisplayMode.MARKER ->{
                drawMarker(input, spikeResults.pixelPoint, Scalar(255.0, 0.0, 0.0))
                input
            }
            DisplayMode.RAW_SPIKE_MASK -> {
                drawMarker(input, spikeResults.pixelPoint, Scalar(255.0, 0.0, 0.0))
                spikeMask.processFrame(input)
            }
            DisplayMode.DENOISED_SPIKE_MASK -> denoisedSpikeMask.processFrame(input)

        }
    }

    override fun onViewportTapped() {
        val modes = enumValues<DisplayMode>()
        val nextOrdinal = (displayMode.ordinal + 1) % modes.size
        displayMode = modes[nextOrdinal]
    }

}