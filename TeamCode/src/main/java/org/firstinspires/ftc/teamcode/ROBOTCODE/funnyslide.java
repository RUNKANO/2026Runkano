package org.firstinspires.ftc.teamcode.ROBOTCODE;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.ftcrobotcontroller.BuildConfig;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp
public class funnyslide extends LinearOpMode {
    private DcMotorEx slide, slide2;
    double slidePower;

    double slide2Power;
    int slidePosition;
    int slide2Position;


    @Override

    public void runOpMode() {



        slide = hardwareMap.get(DcMotorEx.class, "slide");
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide2 = hardwareMap.get(DcMotorEx.class, "slide2");
        slide2.setTargetPosition(0);
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {





            if(gamepad1.rightBumperWasPressed()) slidePower = 1;
            if(gamepad1.dpadUpWasPressed()) slide2Position += 1500;
            if(gamepad1.leftBumperWasPressed()) slidePower = 0;
            if(gamepad1.dpadDownWasPressed()) slide2Position -= 1500;
            if(gamepad1.yWasPressed()) slidePosition += 1500;
            if(gamepad1.aWasPressed()) slidePosition -= 1500;
            if(gamepad1.xWasPressed()) {
                slidePosition = 0;
                slide2Position = 0;
            }
            if(gamepad1.bWasPressed()) slidePosition = 4500;
            if(gamepad1.dpadRightWasPressed()) slidePosition = 0;
            if(gamepad1.dpadLeftWasPressed()) slide2Position = 4500;
            slide.setTargetPosition(slidePosition);
            slide2.setTargetPosition(slide2Position);
            slide.setPower(slidePower);
            slide2.setPower(slidePower);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("Read Slide Position", slide.getCurrentPosition());
            telemetry.addData("Read Slide2 Position", slide2.getCurrentPosition());
            telemetry.addData("Slide Power", slidePower);
            telemetry.addData("Slide2 Power", slide2Power);
            telemetry.addData("Slide Target Position", slide.getTargetPosition());
            telemetry.addData("Slide2 Target Position", slide2.getTargetPosition());
            telemetry.update();
        }
    }}




