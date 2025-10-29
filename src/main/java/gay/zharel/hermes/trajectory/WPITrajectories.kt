package gay.zharel.hermes.trajectory

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.kinematics.ChassisAccelerations
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.math.trajectory.TrajectorySample
import edu.wpi.first.units.measure.Time
import gay.zharel.hermes.geometry.Acceleration2d
import gay.zharel.hermes.geometry.PoseVelocity2d
import gay.zharel.hermes.trajectories.TimeTrajectory
import kotlin.time.Duration

class HermesTrajectorySample(
    timestamp: Time,
    pose: Pose2d,
    velocity: ChassisSpeeds,
    acceleration: ChassisAccelerations
) : TrajectorySample<HermesTrajectorySample>(timestamp, pose, velocity, acceleration) {
    constructor(sample: TrajectorySample<*>) : this(
        sample.timestamp,
        sample.pose,
        sample.velocity,
        sample.acceleration
    )

    constructor(
        timestamp: Duration,
        pose: HPose2d,
        velocity: PoseVelocity2d,
        acceleration: Acceleration2d
    ) : this(
        timestamp.seconds,
        pose.wpilib,
        velocity.wpilib,
        acceleration.wpilib
    )

    /**
     * Transforms the pose of this sample by the given transform.
     *
     * @param transform The transform to apply to the pose.
     * @return A new sample with the transformed pose.
     */
    override fun transform(transform: Transform2d): HermesTrajectorySample = copy(
        pose = pose + transform
    )

    /**
     * Transforms this sample to be relative to the given pose.
     *
     * @param other The pose to make this sample relative to.
     * @return A new sample with the relative pose.
     */
    override fun relativeTo(other: Pose2d): HermesTrajectorySample = copy(
        pose = pose.relativeTo(other)
    )

    /**
     * Creates a new sample with the given timestamp.
     *
     * @param timestamp The new timestamp.
     * @return A new sample with the given timestamp.
     */
    override fun withNewTimestamp(timestamp: Time): HermesTrajectorySample = copy(
        timestamp = timestamp
    )

    internal fun copy(
        timestamp: Time = this.timestamp,
        pose: Pose2d = this.pose,
        velocity: ChassisSpeeds = this.velocity,
        acceleration: ChassisAccelerations = this.acceleration
    ) = HermesTrajectorySample(timestamp, pose, velocity, acceleration)
}

class HermesTrajectory internal constructor(
    samples: Array<HermesTrajectorySample>
) : Trajectory<HermesTrajectorySample>(samples) {
    /**
     * Interpolates between two samples. This method must be implemented by subclasses to provide
     * drivetrain-specific interpolation logic.
     *
     * @param start The starting sample.
     * @param end The ending sample.
     * @param t The interpolation parameter between 0 and 1.
     * @return The interpolated sample.
     */
    override fun interpolate(
        start: HermesTrajectorySample,
        end: HermesTrajectorySample,
        t: Double
    ): HermesTrajectorySample {
        return HermesTrajectorySample(TrajectorySample.kinematicInterpolate(start, end, t))
    }

    /**
     * Transforms all poses in the trajectory by the given transform. This is useful for converting a
     * robot-relative trajectory into a field-relative trajectory. This works with respect to the
     * first pose in the trajectory.
     *
     * @param transform The transform to transform the trajectory by.
     * @return The transformed trajectory.
     */
    override fun transformBy(transform: Transform2d?): HermesTrajectory {
        val firstPose = start().pose
        val transformedFirstPose = firstPose.transformBy(transform)

        val transformedFirstSample =
            HermesTrajectorySample(
                start().timestamp, transformedFirstPose, start().velocity, start().acceleration
            )

        val transformedSamples = samples.asSequence()
                .drop(1)
                .map{ sample ->
                    sample.copy(
                        pose = transformedFirstPose + (sample.pose - firstPose)
                    )
                }

        return HermesTrajectory((sequenceOf(transformedFirstSample) + transformedSamples).toTypedArray())
    }

    /**
     * Concatenates this trajectory with another trajectory. If the other trajectory is empty, this
     * trajectory is returned. To work correctly, the other trajectory should start at the end of this
     * trajectory.
     *
     * @param other the other trajectory to concatenate with this one.
     * @return a new trajectory that is the concatenation of this trajectory and the other trajectory.
     */
    override fun concatenate(other: Trajectory<HermesTrajectorySample>): HermesTrajectory {
        val fixedOtherSamples = other.samples.asSequence()
            .map { it.copy(timestamp = it.timestamp + duration) }

        return HermesTrajectory((this.samples.asSequence() + fixedOtherSamples).toTypedArray())
    }

    /**
     * Returns a new trajectory that is relative to the given pose. This is useful for converting a
     * field-relative trajectory into a robot-relative trajectory. The returned trajectory will have
     * the same timestamps, velocities, and accelerations as the original trajectory, but the poses
     * will be relative to the given pose.
     *
     * @param other the pose to which the trajectory should be relative. This is typically the robot's
     * starting pose.
     * @return a new trajectory that is relative to the given pose.
     */
    override fun relativeTo(other: Pose2d) =
        HermesTrajectory(samples.map { it.relativeTo(other) }.toTypedArray())

    companion object {
        @JvmStatic
        fun fromTimeTrajectory(trajectory: TimeTrajectory): HermesTrajectory {
            val samples = trajectory.profile.times.map { time ->
                val state = trajectory[time]
                HermesTrajectorySample(
                    time.seconds,
                    state.pose.wpilib,
                    state.vel.wpilib,
                    state.accel.wpilib
                )
            }

            return HermesTrajectory(samples.toTypedArray())
        }
    }
}

inline fun <reified T> Sequence<T>.toTypedArray() = toList().toTypedArray()