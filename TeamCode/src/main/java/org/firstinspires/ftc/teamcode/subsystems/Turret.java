package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Config
public class Turret {
    private final Servo turretServo1;
    private final Servo turretServo2;
    private final Telemetry telemetry;
    public Turret(Robot robot) {
        turretServo1 = robot.hardwareMap.get(Servo.class, "turret1");
        turretServo2 = robot.hardwareMap.get(Servo.class, "turret2");
        telemetry = robot.telemetry;
    }
    public void turretToPos(double position){
        turretServo1.setPosition(position);
        turretServo2.setPosition(position);
    }


}
