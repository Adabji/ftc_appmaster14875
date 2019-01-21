package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    private Servo leftIntakeFlipper;
    private Servo rightIntakeFlipper;
    private Servo landerFlipper;
    private CRServo intakeCR;
    private DcMotor extension;
    private TouchSensor topLimit;
    private TouchSensor bottomLimit;
    private TouchSensor inLimit;
    private int extensionReset;


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

        rightIntakeFlipper = hardwareMap.servo.get("rightIntakeFlipper");
        leftIntakeFlipper = hardwareMap.servo.get("leftIntakeFlipper");

        landerFlipper = hardwareMap.servo.get("landerFlipper");
        intakeCR = hardwareMap.crservo.get("intakeCR");

        extension = hardwareMap.dcMotor.get("extension");
        extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        topLimit = hardwareMap.touchSensor.get("topLimit");
        bottomLimit = hardwareMap.touchSensor.get("bottomLimit");

        phoneMount = hardwareMap.servo.get("phoneMount");

        inLimit = hardwareMap.touchSensor.get("inLimit");


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

            //down position of flipper
            if (gamepad2.a) {
                rightIntakeFlipper.setPosition(0.77);
                leftIntakeFlipper.setPosition(0.77);
                Thread.sleep(500);
                rightIntakeFlipper.setPosition(0.87);
                leftIntakeFlipper.setPosition(0.87);
            }

            //up position of flipper
            if (gamepad2.y) {
<<<<<<< HEAD
                leftIntakeFlipper.setPosition(0.24);
                rightIntakeFlipper.setPosition(0.24);
=======
                leftIntakeFlipper.setPosition(0.31);
                rightIntakeFlipper.setPosition(0.31);

>>>>>>> c1eedbc2dc02892dacb5b71ece1f8682558c07cf
            }


            //mid position of flipper
            if (gamepad2.x) {
                leftIntakeFlipper.setPosition(0.57);
                rightIntakeFlipper.setPosition(0.57);


            }
            if (gamepad2.b) {
<<<<<<< HEAD
                leftIntakeFlipper.setPosition(0.5);
                rightIntakeFlipper.setPosition(0.5);
=======
                leftIntakeFlipper.setPosition(0.57);
                rightIntakeFlipper.setPosition(0.57
                );

>>>>>>> c1eedbc2dc02892dacb5b71ece1f8682558c07cf

            }
            //lander flipper up
            if (gamepad1.right_bumper) {
                landerFlipper.setPosition(0.55);

            }
            //lander flipper down
            if (gamepad1.left_bumper) {
                landerFlipper.setPosition(0.05);
            }
            if (gamepad1.a) {
                //in
                intakeCR.setPower(-0.9);
            }
            if (gamepad1.y) {
                //out
                intakeCR.setPower(0.8);
            }
            if (gamepad1.x) {
                //stop
                intakeCR.setPower(0);
            }
            if (gamepad1.b) {
                //stop
                intakeCR.setPower(0);
            }
            //extend extension slides
            if (inLimit.isPressed()) {
                extension.setPower(0);
                Thread.sleep(500);
                extensionReset = extension.getCurrentPosition();
            }
                if (gamepad2.right_bumper) {
                    extension.setPower(1.0);
                } else {
                    if (gamepad2.left_bumper) {
                        extension.setPower(-1.0);
                    } else {
                        extension.setPower(0);
                    }
                }
            if (extension.getCurrentPosition() - extensionReset > 4500){
                extension.setPower(0);
                Thread.sleep(500);
                if (gamepad2.right_bumper) {
                    extension.setPower(1.0);
                } else {
                    if (gamepad2.left_bumper) {
                        extension.setPower(-1.0);
                    } else {
                        extension.setPower(0);
                    }
                }
            }

               /* if (gamepad2.right_bumper) {
                    extension.setPower(1.0);
                } else {
                    if (gamepad2.left_bumper) {
                        extension.setPower(-1.0);
                    } else {
                        extension.setPower(0);
                    }
                }*/

                //limit switch will stop lift if it is too high or low; prevents jamming
                if (bottomLimit.isPressed()) {
                    lift1.setPower(1);
                    lift2.setPower(1);
                    Thread.sleep(100);
                    stopLift();
                    lift1.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
                    lift2.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
                } else {
                    if (topLimit.isPressed()) {
                        lift1.setPower(-1);
                        lift2.setPower(-1);
                        Thread.sleep(100);
                        stopLift();
                        lift1.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
                        lift2.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
                    } else {
                        lift1.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
                        lift2.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
                    }
                }
            }
        }
        private void stopLift () throws InterruptedException {
            lift1.setPower(0);
            lift2.setPower(0);
            Thread.sleep(500);
        }
    }