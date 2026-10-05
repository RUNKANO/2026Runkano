package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.Robot;

import static com.pedropathing.ivy.commands.Commands.*;
@Config
public class Intake {
    private boolean slowMode = false;
    private Mode mode = Mode.OFF;
    public static double fastPower = -1;
    public static double offPower = 0;
    public static double reversePower = 1;
    public static double shortReverseTimeMs = 150;

    private final DcMotorEx intakeMotor1, intakeMotor2;

    private final Telemetry telemetry;

    public Intake(Robot robot) {
        intakeMotor1 = robot.hardwareMap.get(DcMotorEx.class, "intake1");
        intakeMotor2 = robot.hardwareMap.get(DcMotorEx.class, "intake2");
        intakeMotor2.setDirection(DcMotorEx.Direction.REVERSE);
        telemetry = robot.telemetry;
    }

    public Command on() {
        return instant(() -> mode = Mode.ON).requiring(intakeMotor1,intakeMotor2);
    }

    public Command off() {
        return instant(() -> mode = Mode.OFF).requiring(intakeMotor1,intakeMotor2);
    }

    public Command reverse() {
        return instant(() -> mode = Mode.REVERSE).requiring(intakeMotor1, intakeMotor2);
    }

    public Command shortReverse() {
        return reverse().then(waitMs(shortReverseTimeMs)).then(on());
    }

    public Command toggle() {
        return conditional(() -> mode == Mode.OFF, on(), off());
    }

    public void slowDown() {
        slowMode = true;
    }

    public void speedUp() {
        slowMode = false;
    }
    public void setPower(double power){
        intakeMotor1.setPower(power);
        intakeMotor2.setPower(power);

    }
    public Command periodic() {
        return infinite(() -> {
            switch (mode) {
                case ON:
                    setPower(fastPower);
                    break;
                case OFF:
                    setPower(offPower);
                    break;
                case REVERSE:
                    setPower(reversePower);
                    break;
            }

            telemetry.addData("Intake 1 Current", intakeMotor1.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("Intake 2 Current", intakeMotor2.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("Intake Velocity", intakeMotor1.getVelocity());
        });
    }

    enum Mode {
        ON,
        OFF,
        REVERSE
    }
}
