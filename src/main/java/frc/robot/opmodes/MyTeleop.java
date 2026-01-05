package frc.robot.opmodes;

import frc.robot.Robot;
import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

@Teleop(name="Peak Teleop")
public class MyTeleop extends PeriodicOpMode {
    private final Robot robot;

    public MyTeleop(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void periodic() {
        Scheduler.getDefault().run();
    }
}
