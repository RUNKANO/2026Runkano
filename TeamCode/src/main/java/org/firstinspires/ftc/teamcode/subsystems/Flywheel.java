package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.Robot;
@Config
public class Flywheel {
    private final DcMotorEx flywheelMotor;
    private final Telemetry telemetry;
    public double target = 2000; //find what this needs to be later, and we can create a function or array for it later
    private boolean on = false;
    public static int velocityTolerance = 25;
    public Flywheel(Robot robot){
        flywheelMotor = robot.hardwareMap.get(DcMotorEx.class, "flywheel");

        telemetry = robot.telemetry;
    }
    private double getVelocity() {
        return flywheelMotor.getVelocity();
    }
    public void turnOn() {
        on = true;
    }

    public void turnOff() {
        on = false;
    }

    public void toggle() {
        on = !on;
        if (on) turnOn();
        else turnOff();
    }
    public void setTarget(double target){
        this.target = target;
    }
    public boolean atTarget(){
        return Math.abs(target - getVelocity()) <= velocityTolerance;
    }
    public Command periodic() {
        return infinite(() -> {
            if (on) {
                flywheelMotor.setVelocity(target);
            } else {
               flywheelMotor.setVelocity(0);
            }

            telemetry.addData("Flywheel Velocity", getVelocity());
            telemetry.addData("Flywheel Target", target);
        });
    }
}
