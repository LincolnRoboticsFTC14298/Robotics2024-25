package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.Vector2d

object FieldConfig {

    const val tileSize = 23.5 // unnecessary?

    const val spikeDiameter = 4.0 //in
    const val spikeHeight = 3.5 //in

    const val backdropAprilTagHeight = 2.0 //in
    const val audienceSmallAprilTagHeight = 2.0 //in
    const val audienceBigAprilTagHeight = 5.0 //in



    /**
     * All of the apriltags on the field and their respective ids.
     */
    enum class AprilTagResult(var id: Int, var tagSize: Double) {
        BACKDROP_LEFT_BLUE(1, 2.0),
        BACKDROP_MIDDLE_BLUE(2, 2.0),
        BACKDROP_RIGHT_BLUE(3, 2.0),

        BACKDROP_LEFT_RED(4, 2.0),
        BACKDROP_MIDDLE_RED(5, 2.0),
        BACKDROP_RIGHT_RED(6, 2.0),

        AUDIENCE_WALL_BIG_BLUE(10, 5.0),
        AUDIENCE_WALL_SMALL_BLUE(9, 2.0),

        AUDIENCE_WALL_BIG_RED(7, 5.0),
        AUDIENCE_WALL_SMALL_RED(8, 2.0);

        companion object {
            fun find(id: Int): AprilTagResult? = AprilTagResult.values().find { it.id == id }
        }
    }


}