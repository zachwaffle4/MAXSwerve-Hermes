package gay.zharel.hermes.trajectory

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint
import gay.zharel.hermes.control.SwerveKinematics
import gay.zharel.hermes.control.WheelVelConstraint
import gay.zharel.hermes.geometry.RobotState
import gay.zharel.hermes.geometry.Vector2d
import gay.zharel.hermes.paths.PosePath
import gay.zharel.hermes.profiles.AccelConstraint
import gay.zharel.hermes.profiles.AngularVelConstraint
import gay.zharel.hermes.profiles.MinVelConstraint
import gay.zharel.hermes.profiles.ProfileAccelConstraint
import gay.zharel.hermes.profiles.VelConstraint
import kotlin.math.absoluteValue
import kotlin.math.pow

class SwerveKinematicsConstraints @JvmOverloads constructor(
    kinematics: SwerveDriveKinematics,
    maxTransVel: Double,
    maxTransAccel: Double,
    minTransAccel: Double = maxTransAccel,
) {
    val velConstraint: VelConstraint
    val accelConstraint: AccelConstraint

    init {
        require(maxTransVel > 0) { "Maximum translational velocity must be positive." }
        require(maxTransAccel > 0) { "Maximum translational acceleration must be positive." }
        require(minTransAccel > 0) { "Minimum translational acceleration must be positive." }

        val wpilibConstraint = SwerveDriveKinematicsConstraint(kinematics, maxTransVel)

        velConstraint = VelConstraint { robotState, path, s ->

            wpilibConstraint.getMaxVelocity(
                robotState.pose.wpilib,
                path.curvatureAt(s),
                robotState.vel.linearVel.norm()
            )
        }

        accelConstraint = ProfileAccelConstraint(
            -minTransAccel,
            maxTransAccel
        )
    }
}

infix fun Vector2d.cross(other: Vector2d) = this.x * other.y - this.y * other.x

fun PosePath.curvatureAt(s: Double): Double {
    val state = RobotState.fromDualPose(this[s, 3])
    val numerator = state.vel.linearVel cross state.accel.linearAcc
    val denominator = state.vel.linearVel.norm().pow(3)

    return if (denominator.absoluteValue < 1e-9) 0.0 else numerator / denominator
}