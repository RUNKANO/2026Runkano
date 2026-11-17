package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.Robot;
@Config
public class Flywheel {
    private final DcMotorEx flywheelMotor, flywheelMotor2;
    private final Telemetry telemetry;
    public double target = 2200; //find what this needs to be later, and we can create a function or array for it later
    private boolean on = false;
    public static int velocityTolerance = 80;
    public static double kP = 0.004;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0.00036;
    PIDFController pidfController = new PIDFController(kP,kI,kD,kF);
    public Flywheel(Robot robot){
        flywheelMotor = robot.hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelMotor2 = robot.hardwareMap.get(DcMotorEx.class, "flywheel2");
        flywheelMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry = robot.telemetry;
    }
    private void setPower(){
         flywheelMotor.setPower(pidfController.calculate(target, getVelocity()));
         flywheelMotor2.setPower(pidfController.calculate(target, getVelocity()));
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
                setPower();
            } else {
                flywheelMotor.setPower(0);
                flywheelMotor2.setPower(0);
            }

            telemetry.addData("Flywheel Velocity", getVelocity());
            telemetry.addData("Flywheel Target", target);
            telemetry.addData("Flywheel Milliamps", flywheelMotor.getCurrent(CurrentUnit.MILLIAMPS)+flywheelMotor2.getCurrent(CurrentUnit.MILLIAMPS));
        });
    }
}
