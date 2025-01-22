package org.firstinspires.ftc.teamcode.vision

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.FieldConfig
import org.firstinspires.ftc.teamcode.subsystems.Vision
import org.firstinspires.ftc.teamcode.vision.modulelib.InputModule
import org.firstinspires.ftc.teamcode.vision.modulelib.ModularPipeline
import org.firstinspires.ftc.teamcode.vision.modules.*
import org.firstinspires.ftc.teamcode.vision.modules.features.*
import org.firstinspires.ftc.teamcode.vision.modules.scorers.*
import org.opencv.core.Mat
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.opencv.imgproc.Imgproc.drawContours

open class ConeDetectionPipeline(
        private var displayMode: DisplayMode = DisplayMode.ALL_CONTOURS,
        camera: Vision.Companion.CameraData,
        var telemetry: Telemetry?
) : ModularPipeline() {

    //private val camMat = Mat()
    //private val distCoeffs = Mat()

    enum class DisplayMode {
        RAW_CAMERA_INPUT,
        RAW_GREEN_MASK,
        DENOISED_MASK,
        UNFILTERED_CONTOURS,
        CONE_FILTERED_CONTOURS,
        ALL_CONTOURS
    }

    // Modules //
    private val inputModule = InputModule()
    //private val undistort = UndistortLens(inputModule, camMat, distCoeffs)
    private val labColorSpace = ColorConverter(inputModule, Imgproc.COLOR_RGB2Lab)
    private val whiteMask = Filter(labColorSpace, Scalar(45.0, -128.0, 68.0), Scalar(100.0, 0.0, .0))
    private val yellowMask = Filter(labColorSpace, Scalar(45.0, -128.0, 68.0), Scalar(255.0, 105.0, 255.0))
    private val greenMask = Filter(labColorSpace, Scalar(45.0, -128.0, 68.0), Scalar(255.0, 105.0, 255.0))
    private val purpleMask = Filter(labColorSpace, Scalar(45.0, -128.0, 68.0), Scalar(255.0, 105.0, 255.0))
    //private val combinedMask = redMask //TODO change back once done testing
    private val denoisedConeMask = Denoise(greenMask, 5, 5, 3, 3)
    private val rawConeContours = Contours(denoisedConeMask)

    // Single Cone Scorer //
    private val singleConeConvexityScorer = DiffSquaredScorer(Convexity() , 0.942, 13.3)
    private val singleConeExtentScorer = DiffSquaredScorer(Extent(), 0.668, 4.06)
    private val singleConeSolidityScorer = DiffSquaredScorer(Solidity() , 0.901, 38.9)
    private val singleConeAspectRatioScorer = DiffSquaredScorer(AspectRatio() , 1.369, 0.23)
    private val singleConeContours = FilterContours(rawConeContours, 0.05, singleConeConvexityScorer + singleConeExtentScorer + singleConeSolidityScorer + singleConeAspectRatioScorer)

    // Single Color Mask and Single Cone Overlap //


    // Results Modules //
    private val singleConeResultsModule = ContourResults(singleConeContours, camera, FieldConfig.spikeDiameter)


    // Data we care about and wish to access
    var singleConeResults = listOf<ContourResults.AnalysisResult>()

    init {
        addEndModules(singleConeResultsModule)
    }

    override fun processFrameForCache(input: Mat) : Mat {

        // Get the data we want (yipee)
        singleConeResults = singleConeResultsModule.processFrame(input)


        // Telemetry for Testing //
        telemetry?.addData("displaymode", displayMode)

        telemetry?.addLine("mean, variance")

        //telemetry.addData("areaResults", poleArea.areaResultsList().sorted().toString())

        // Display //
        return when (displayMode) {
            DisplayMode.RAW_CAMERA_INPUT -> input
            DisplayMode.RAW_GREEN_MASK -> greenMask.processFrame(input)
            DisplayMode.DENOISED_MASK -> denoisedConeMask.processFrame(input)
            DisplayMode.UNFILTERED_CONTOURS ->{
                drawContours(input, rawConeContours.processFrame(input), -1, Scalar(0.0, 255.0, 0.0), 1)
                input
            }
            DisplayMode.CONE_FILTERED_CONTOURS ->{
                drawContours(input, singleConeContours.processFrame(input), -1, Scalar(0.0, 255.0, 0.0), 1)
                input
            }

            DisplayMode.ALL_CONTOURS -> {
                drawContours(input, rawConeContours.processFrame(input), -1, Scalar(0.0, 255.0, 0.0), 1) //Green for other cone contours

                input
            }
        }
    }

    override fun onViewportTapped() {
        val modes = enumValues<DisplayMode>()
        val nextOrdinal = (displayMode.ordinal + 1) % modes.size
        displayMode = modes[nextOrdinal]
    }


}