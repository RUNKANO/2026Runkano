package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
@TeleOp
public class PFTuning extends OpMode {
    public DcMotorEx fly;

    public double highVelocity = 2800;
    public double lowVelocity = 1400;
    double curTargetVelocity = highVelocity;

    double P = 0;
    double F = 0;
    double[] stepSize = {10.0, 1.0, .1, .01};
    int stepIndex = 0;

    @Override
    public void init() {
        fly = hardwareMap.get(DcMotorEx.class, "flywheel");
        fly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        fly.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("init complete");
    }

    @Override
    public void loop() {
        if(gamepad1.yWasPressed()){
            if(curTargetVelocity == highVelocity){
                curTargetVelocity = lowVelocity;
            }else if(curTargetVelocity == lowVelocity){
                curTargetVelocity = highVelocity;
            }
        }
        if(gamepad1.bWasPressed()){
            stepIndex = (stepIndex+1)% stepSize.length;
        }
        if(gamepad1.dpadUpWasPressed()) {
            P += stepSize[stepIndex];
        }
        if(gamepad1.dpadDownWasPressed()){
            P -= stepSize[stepIndex];
        }
        if(gamepad1.dpadLeftWasPressed()){
            F -= stepSize[stepIndex];
        }
        if(gamepad1.dpadRightWasPressed()){
            F += stepSize[stepIndex];
        }
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        fly.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        fly.setVelocity(curTargetVelocity);

        double curVelocity = fly.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("Filler line :3 :3 :3");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSize[stepIndex]);
        telemetry.update();
    }
}
