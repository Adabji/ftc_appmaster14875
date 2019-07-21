package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;
import java.util.TimerTask;

// TeleOp Program for Mecanum Drive
@TeleOp(name = "Mecanum TeleOp", group = "TeleOp")
public class TeleOpMecanumDrive extends LinearOpMode {

    // Motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        // Wait for the game to start
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "waiting for start command...");
            telemetry.update();
        }

        while (opModeIsActive()) {
            telemetry.addData("Status", "received start command.");
            telemetry.update();

            // setting power of all motors to gamepad1's left stick
            frontLeft.setPower(gamepad1.left_stick_y - -gamepad1.left_stick_x);
            frontRight.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);
            backLeft.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);
            backRight.setPower(gamepad1.left_stick_y - -gamepad1.left_stick_x);

            /* strafe left
            if(gamepad1.left_stick_x < 0) {
                frontLeft.setPower(gamepad1.left_stick_x);
                backLeft.setPower(-gamepad1.left_stick_x);

                frontRight.setPower(-gamepad1.left_stick_x);
                backRight.setPower(gamepad1.left_stick_x);
            }

            // strafe right
            if(gamepad1.left_stick_x > 0) {
                frontLeft.setPower(gamepad1.left_stick_x);
                backLeft.setPower(-gamepad1.left_stick_x);

                frontRight.setPower(-gamepad1.left_stick_x);
                backRight.setPower(gamepad1.left_stick_x);
            }*/
        }
    }
}
