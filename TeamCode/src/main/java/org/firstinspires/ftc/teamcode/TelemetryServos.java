package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "TelemetryServos", group = "TeleOp")
@Disabled
public class TelemetryServos extends LinearOpMode {
    private Servo leftArm;
    private Servo rightArm;

    @Override
    public void runOpMode() throws InterruptedException {
        leftArm = hardwareMap.servo.get("leftArm");
        rightArm = hardwareMap.servo.get("rightArm");
        waitForStart();
        leftArm.setPosition(-0.1);
        rightArm.setPosition(0.9);
        Thread.sleep(1000);
        leftArm.setPosition(0.5);
        rightArm.setPosition(0.3);
        Thread.sleep(100);
        while (opModeIsActive()) {
            telemetry.addData("leftArmPos", leftArm.getPosition());
            telemetry.update();
            Thread.sleep(100);
            telemetry.addData("rightArmPos", rightArm.getPosition());
            telemetry.update();
        }

    }
}