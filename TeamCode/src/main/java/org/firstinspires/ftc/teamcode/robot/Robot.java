package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.*;
public class Robot {
    public final HardwareMap hardwareMap;
    public final Telemetry telemetry;
    public final Intake intake;
    public final Blocker blocker;

    public final Flywheel flywheel;


    public Robot(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
        telemetry = new MultipleTelemetry(
                opMode.telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );
        blocker = new Blocker(this);
        intake = new Intake(this);
        flywheel = new Flywheel(this);
    }
}
