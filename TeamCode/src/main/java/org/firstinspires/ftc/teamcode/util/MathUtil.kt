package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import kotlin.math.abs
import kotlin.math.max

/**
 * @return Normalized vector.
 */
//fun Vector2d.normalize(): Vector2d = this.div(this.norm())
//fun Vector2d.normalize(): Vector2d = this.div(max(this.norm(), 1.0))
fun Vector2d.normalize(): Vector2d {return if(this.norm() == 0.0) { this } else {this.div(this.norm())}}

const val EPSILON = 1e-6 //TODO check value of epsilon
infix fun Double.epsilonEquals(other: Double) = abs(this - other) < EPSILON