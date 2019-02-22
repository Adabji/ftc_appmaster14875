package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;


//TeleOp program for HDrive
@TeleOp(name = "HDrive TeleOp", group = "TeleOp")
public class TeleOpHDrive extends LinearOpMode {

    //Declare motors
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    private DcMotor strafingRight;
    private DcMotor strafingLeft;
    private DcMotor lift1;
    private DcMotor lift2;
    private Servo phoneMount;
    private Servo leftSampleArm;
    private DcMotor extension;
    private TouchSensor topLimit;
    private TouchSensor bottomLimit;
    private TouchSensor inLimit;
    private int extensionReset;
    private DcMotor intake;
    private Servo liftServo1;
    private Servo liftServo2;


    @Override
    public void runOpMode() throws InterruptedException {

//Initialize motors
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");

        lift1 = hardwareMap.dcMotor.get("lift1");
        lift2 = hardwareMap.dcMotor.get("lift2");

//Strafing motors move together to make the robot go left/right without turning
        strafingRight = hardwareMap.dcMotor.get("strafingRight");
        strafingLeft = hardwareMap.dcMotor.get("strafingLeft");

        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        leftSampleArm = hardwareMap.servo.get("leftSampleArm");

        extension = hardwareMap.dcMotor.get("extension");
        extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        topLimit = hardwareMap.touchSensor.get("topLimit");
        bottomLimit = hardwareMap.touchSensor.get("bottomLimit");

        phoneMount = hardwareMap.servo.get("phoneMount");

        inLimit = hardwareMap.touchSensor.get("inLimit");

        intake = hardwareMap.dcMotor.get("intake");

        liftServo1 = hardwareMap.servo.get("liftServo1");

        liftServo2 = hardwareMap.servo.get("liftServo2");


//Wait for the game to start
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "waiting for start command...");
            telemetry.update();
        }
        while (opModeIsActive()) {
            //setting power of back motors
            motorBackLeft.setPower(-gamepad1.left_stick_y);
            motorBackRight.setPower(-gamepad1.right_stick_y);
            phoneMount.setPosition(0.43);

            //setting power of strafing motors
            strafingLeft.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            strafingRight.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

            //setting positions of servo arms
            leftSampleArm.setPosition(0.3);

            if (gamepad1.left_bumper) {
                liftServo1.setPosition(0.96);
                Thread.sleep(100);
                liftServo2.setPosition(0.72);
            }
            if (gamepad1.right_bumper) {
                liftServo1.setPosition(0);
                Thread.sleep(150);
                liftServo2.setPosition(0.3);
            }
            if (gamepad1.a) {
                intake.setPower(1);
            }

            if (gamepad1.y) {
                intake.setPower(-1);
            }

            if (gamepad1.x) {
                intake.setPower(0);
            }

            if (gamepad1.b) {
                intake.setPower(0);
            }
            //extend extension slides
            if (gamepad2.right_bumper) {
                extension.setPower(1);
            } else if (gamepad2.left_bumper) {
                if (inLimit.isPressed()) {
                    extension.setPower(0);
                } else {
                    extension.setPower(-1);
                }
            } else {
                extension.setPower(0);

            }
            //limit switch will stop lift if it is too high or low; prevents jamming
            if (bottomLimit.isPressed()) {
                lift1.setPower(1);
                lift2.setPower(1);
                Thread.sleep(100);
                lift1.setPower(0);
                lift2.setPower(0);
                Thread.sleep(500);
                lift1.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
                lift2.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
            } else {
                if (topLimit.isPressed()) {
                    lift1.setPower(-1);
                    lift2.setPower(-1);
                    Thread.sleep(50);
                    lift1.setPower(0);
                    lift2.setPower(0);
                    Thread.sleep(500);
                    lift1.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
                    lift2.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
                } else {
                    lift1.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
                    lift2.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
                }

            }


            }
    }
}


