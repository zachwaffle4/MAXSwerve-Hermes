package gay.zharel.hermes.trajectory

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisAccelerations
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Unit
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.InchesPerSecond
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Time
import gay.zharel.hermes.geometry.Acceleration2d
import gay.zharel.hermes.geometry.PoseVelocity2d
import gay.zharel.hermes.geometry.Vector2d
import kotlin.time.Duration
import kotlin.time.DurationUnit

fun <U : Unit> Measure<U>.into(unit: U) = this.`in`(unit)

inline val Double.inches: Distance get() = Inches.of(this)
inline val Double.ips: LinearVelocity get() = InchesPerSecond.of(this)
inline val Double.meters: Distance get() = Meters.of(this)
inline val Double.mps: LinearVelocity get() = MetersPerSecond.of(this)
inline val Double.radians: Angle get() = Radians.of(this)
inline val Double.radps: AngularVelocity get() = RadiansPerSecond.of(this)

inline val Distance.inches get() = this.into(Inches)
inline val LinearVelocity.ips get() = this.into(InchesPerSecond)
inline val Distance.meters get() = this.into(Meters)
inline val LinearVelocity.mps get() = this.into(MetersPerSecond)
inline val Angle.radians get() = this.into(Radians)
inline val AngularVelocity.radps get() = this.into(RadiansPerSecond)

inline val Double.seconds: Time get() = Seconds.of(this)
inline val Duration.seconds: Time get() = Seconds.of(toDouble(DurationUnit.SECONDS))

typealias HPose2d = gay.zharel.hermes.geometry.Pose2d
typealias HRotation2d = gay.zharel.hermes.geometry.Rotation2d

@get:JvmName("wpilib")
inline val Vector2d.wpilib get() = Translation2d(x.inches, y.inches)
@get:JvmName("hermes")
inline val Translation2d.hermes get() = Vector2d(x.meters.inches, y.meters.inches)

@get:JvmName("wpilib")
inline val HRotation2d.wpilib get() = Rotation2d(real, imag)
@get:JvmName("hermes")
inline val Rotation2d.hermes get() = HRotation2d(cos, sin)

@get:JvmName("wpilib")
inline val HPose2d.wpilib get() = Pose2d(position.wpilib, heading.wpilib)
@get:JvmName("hermes")
inline val Pose2d.hermes get() = HPose2d(translation.hermes, rotation.hermes)

@get:JvmName("wpilib")
inline val PoseVelocity2d.wpilib get() = ChassisSpeeds(linearVel.x.ips, linearVel.y.ips, angVel.radps)
@get:JvmName("hermes")
inline val ChassisSpeeds.hermes get() = PoseVelocity2d(Vector2d(vx.mps.ips, vy.mps.ips), omega)

@get:JvmName("wpilib")
inline val Acceleration2d.wpilib get() = ChassisAccelerations(linearAcc.x.inches.meters, linearAcc.y.inches.meters, angAcc)
@get:JvmName("hermes")
inline val ChassisAccelerations.hermes get() = Acceleration2d(Vector2d(ax.meters.inches, ay.meters.inches), alpha)