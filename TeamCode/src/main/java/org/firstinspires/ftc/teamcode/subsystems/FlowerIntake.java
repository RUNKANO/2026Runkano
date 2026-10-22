package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class FlowerIntake {
    private Mode mode = Mode.UP;
    private final Servo flowerLeft;
    private final Servo flowerRight;
    private final Telemetry telemetry;
    public FlowerIntake(Robot robot){
        flowerRight = robot.hardwareMap.get(Servo.class, "flowerL");
        flowerLeft = robot.hardwareMap.get(Servo.class, "flowerR");
        telemetry = robot.telemetry;
    }

    public void setPosition(double offset){
        flowerRight.setPosition(.5-offset);
        flowerLeft.setPosition(.5+offset);
    }
    public Command up(){
        return instant(() -> mode = Mode.UP);
    }
    public Command down(){
        return instant(()-> mode = Mode.DOWN);
    }
    public Command toggle(){
        return conditional(()-> mode == Mode.DOWN, up(), down());
    }
    public Command periodic(){
        return infinite(()->{
            switch (mode) {
                case UP:
                    setPosition(0);
                    telemetry.addLine("Flower Position Up");
                    break;
                case DOWN:
                    setPosition(.3);
                    telemetry.addLine("Flower Position Down");
                    break;
            }
        });
    }
    enum Mode{
        DOWN,
        UP
    }
}

