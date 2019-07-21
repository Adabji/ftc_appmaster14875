package org.firstinspires.ftc.teamcode;

import android.graphics.drawable.GradientDrawable;
import android.hardware.camera2.CameraDevice;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.imgproc.LineSegmentDetector;

import java.util.concurrent.SynchronousQueue;
import java.util.concurrent.ThreadLocalRandom;

@Autonomous(name = "Auto Cycles Depot", group = "Autonomous")
//Declare motors
public class AutoCyclesDepot extends LinearOpMode {
    private DcMotor lift1, lift2, strafingRight, strafingLeft, motorBackRight, motorBackLeft, extension, intake;
    private Servo sampleArm, liftServo1, liftServo2, flipper1, flipper2, stopper;
    private TouchSensor bottomLimit, topLimit, midLimit;
    boolean center = false, right = false, intakeNow = false, raiseLiftVar = true, intakeRetract = false, intakeUp, lift = true, stop = false;
    double extensionCounter, negExtensionCounter, time2 = 1;
    private GoldAlignDetector detector;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");

        //Set up detector with phone camera
        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();
        detector.alignSize = 1000; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        //Initialize motors
        lift1 = hardwareMap.dcMotor.get("lift1");
        lift2 = hardwareMap.dcMotor.get("lift2");
        strafingRight = hardwareMap.dcMotor.get("strafingRight");
        strafingLeft = hardwareMap.dcMotor.get("strafingLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        sampleArm = hardwareMap.servo.get("sampleArm");

        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        strafingRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        strafingLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extension = hardwareMap.dcMotor.get("extension");
        extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        bottomLimit = hardwareMap.touchSensor.get("bottomLimit");
        topLimit = hardwareMap.touchSensor.get("topLimit");
        intake = hardwareMap.dcMotor.get("intake");
        liftServo1 = hardwareMap.servo.get("liftServo1");

        liftServo2 = hardwareMap.servo.get("liftServo2");


        flipper1 = hardwareMap.servo.get("flipper1");
        flipper2 = hardwareMap.servo.get("flipper2");
        stopper = hardwareMap.servo.get("stopper");
        midLimit = hardwareMap.touchSensor.get("midLimit");
        telemetry.setAutoClear(true);
        detector.enable();

        //waitForStart();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "waiting for start command...");
            flipper1.setPosition(0.15);
            flipper2.setPosition(0.85);
            // liftServo1.setPosition(0.96);
            // liftServo2.setPosition(0.72);
            lift1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            lift2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            //Telemetry returned X-Value for when block is seen in center position
            if (detector.getXPosition() > 500) {
                right = true;
                center = false;
                telemetry.addData("center", center);
                telemetry.addData("right", right);
                telemetry.update();
            }
            if (detector.getXPosition() > 120 && detector.getXPosition() < 260) {
                center = true;
                right = false;
                telemetry.addData("center", center);
                telemetry.addData("right", right);
                telemetry.update();
            }else if (!detector.getAligned()) {
                right = false;
                center = false;
                telemetry.addData("center", center);
                telemetry.addData("right", right);
                telemetry.update();
            }
            /*if (isStopRequested()) {
                requestOpModeStop();
            }*/


            telemetry.addData("center", center);
            telemetry.addData("right", right);
            telemetry.update();

        }
        if (opModeIsActive()) {
            detector.disable();
            liftServo1.setPosition(0.9);    // changed
            liftServo2.setPosition(0.72);
            sampleArm.setPosition(0.8);
            stopper.setPosition(0.4);
            lowerRobot();
            Thread.sleep(500);

            //Code to run if block is seen in center position, if variable center is returned as true
            if (center) {
                sampleCenter();
                moveForwardsIntake(500, 0.5);
                intakeIn();
                retract2(1, 230);
                stopper.setPosition(0.85);
                moveBackwardsIntake(600, 1);
                turnRight(100,0.8);
                teleLift();
                liftWithEncoders(1,2900);
                liftServo1.setPosition(0);
                Thread.sleep(200);
                liftServo2.setPosition(0.15);
                Thread.sleep(1600);
                lowerRobot();
                rotateRight(100,1);
                rotateLeft(100,1);
                Thread.sleep(1000);
                liftServo1.setPosition(0.9);    // changed
                Thread.sleep(700);
                liftServo2.setPosition(0.72);
                Thread.sleep(200);
                rotateLeftSlow(1000,0.5);
                lift = true;
                moveForwardsCycle(1200,0.5);
                rotateLeftSlow(600,1);
                extend(1,1200);
                flipperDown();
                Thread.sleep(500);
                intake.setPower(1);
                Thread.sleep(3000);



            /*rotateLeft(440, 0.5);
            Thread.sleep(300);
            moveForwards(1100, .5);
            Thread.sleep(200);
            rotateLeftSlow(650, .5);
            Thread.sleep(200);
            parkInCraterCenter(300,1);
            Thread.sleep(500);*/

            }

            //Code to run if block is seen in left position, if variables and center are returned as false (implying that left is true)
            if (!right & !center) {
                sampleLeft();
                rotateLeft(190, 0.2);
                Thread.sleep(300);
                moveForwardsIntake(600, 0.5);
                intakeIn();
                retract2(1, 250);
                stopper.setPosition(0.85);
                moveBackwardsIntake(700, 1);
                Thread.sleep(200);
                rotateRight(190, 0.5);
                turnLeft(200,0.8);
                teleLift();
                liftWithEncoders(1,2900);
                liftServo1.setPosition(0);
                liftServo2.setPosition(0.15);
                Thread.sleep(1600);
                lowerRobot();
                turnRight(100,1);
                turnLeft(50,1);
                Thread.sleep(1000);
                liftServo1.setPosition(0.9);    // changed
                Thread.sleep(700);
                liftServo2.setPosition(0.72);
                Thread.sleep(1000);
                rotateLeftSlow(1100,1);
                lift = true;
                moveForwardsCycle(1250,0.5);
                rotateLeftSlow(450,1);
                extend(1,1200);
                flipperDown();
                Thread.sleep(200);
                intake.setPower(1);
                Thread.sleep(3000);


            /*rotateLeft(780, 0.5);
            Thread.sleep(200);
            moveForwards(200, 0.5);
            Thread.sleep(200);
            rotateLeftSlow(400, 0.5);
            Thread.sleep(200);
            moveForwards(300, 0.5);
            Thread.sleep(500);
            parkInCrater();*/


            }
            //Code to run if block is in right position, not visible as an X-Value returned but rather as the condition
            //when both left and center are negated as true conditions
            if (right) {
                sampleRight();
                rotateRight(190, 0.2);
                Thread.sleep(300);
                moveForwardsIntake(600, 0.5);
                intake.setPower(1);
                Thread.sleep(300);
                intakeIn();
                retract2(1, 250);
                stopper.setPosition(0.85);
                moveBackwardsIntake(740, 1);
                Thread.sleep(200);
                rotateLeft(230, 0.5);
                teleLift();
                liftWithEncoders(1,2900);
                liftServo1.setPosition(0);
                Thread.sleep(500);
                liftServo2.setPosition(0.15);
                Thread.sleep(1600);
                lowerRobot();
                rotateRight(100,1);
                rotateLeft(100,1);
                Thread.sleep(1000);
                liftServo1.setPosition(0.9);    // changed
                Thread.sleep(700);
                liftServo2.setPosition(0.72);
                Thread.sleep(400);
                rotateLeftSlow(1050,1);
                lift = true;
                moveForwardsCycle(1250,0.5);
                rotateLeftSlow(450,1);
                extend(1,1200);
                flipperDown();
                Thread.sleep(200);
                intake.setPower(1);
                Thread.sleep(3000);


            /*moveBackwards(900, 0.5);
            Thread.sleep(300);
            rotateLeft(600, 0.5);
            Thread.sleep(300);
            moveForwards(1700, 0.5);
            Thread.sleep(300);
            rotateLeftSlow(350, 0.5);
            parkInCrater();*/
            }
        }
    }

        //Lowers the robot from the lander at the beginning of Autonomous period
        public void lowerRobot () throws InterruptedException {
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            while (!bottomLimit.isPressed()) {
                lift1.setPower(-1);
                lift2.setPower(-1);
                if (bottomLimit.isPressed()) {
                    lift1.setPower(0);
                    lift2.setPower(0);
                }
            }
        }

        public void teleLift() {
            while (topLimit.isPressed()) {
                if (time2 == 1) {
                    lift1.setPower(-1);
                    lift2.setPower(-1);
                    time2 = 2;
                }
            }
            if (time2 == 2) {
                lift1.setPower(-1);
                lift2.setPower(-1);

                time2 = 3;
            }
            if (time2 == 3) {
                lift1.setPower(-1);
                lift2.setPower(-1);
                time2 = 1;
            }
            if (!topLimit.isPressed()) {
                lift1.setPower(0);
                lift2.setPower(0);
            }
        }

        public void turnRight ( int distance, double power) throws InterruptedException {
            strafingRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            strafingLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            strafingRight.setTargetPosition(distance);
            strafingLeft.setTargetPosition(distance);

            strafingRight.setPower(power);
            strafingLeft.setPower(power);

            strafingRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            strafingLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (strafingLeft.isBusy() && strafingRight.isBusy()) {
            }

            strafingRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            strafingLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

        public void turnLeft ( int distance, double power) throws InterruptedException {
            turnRight(-distance, power);
        }

        public void sampleRight () throws InterruptedException {
            moveForwards2(770, .5);
            Thread.sleep(200);
            flipperUp();
            Thread.sleep(200);
            moveBackwards(450, 1);
            retract(1, 1300);
            Thread.sleep(400);
        }

        public void sampleLeft () throws InterruptedException {
            moveForwards2(770, .5);
            Thread.sleep(200);
            flipperUp();
            Thread.sleep(200);
            moveBackwards(450, 1);
            retract(1, 1300);
            Thread.sleep(400);
        }

        public void sampleCenter () throws InterruptedException {
            moveForwards2(850, .5);
            Thread.sleep(200);
            flipperUp();
            Thread.sleep(200);
            moveBackwards(450, 0.5);
            retract(1, 1300);
            Thread.sleep(400);
        }

    public void teamMarker() throws InterruptedException {
        sampleArm.setPosition(0.9);
        Thread.sleep(1000);
        sampleArm.setPosition(0.3);
    }

    public void rotateLeft(int distance, double power) {
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackRight.setTargetPosition(distance);
        motorBackLeft.setTargetPosition(distance);

        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBackRight.setPower(power);
        motorBackLeft.setPower(power);

        while (motorBackRight.isBusy() && motorBackLeft.isBusy()) {
        }

        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void rotateLeftSlow(int distance, double power) {
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setTargetPosition(distance);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setPower(power);
        while (motorBackLeft.isBusy()) {
        }
        motorBackLeft.setPower(0);
        motorBackLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void rotateRightSlow(int distance, double power) {
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setTargetPosition(-distance);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setPower(power);
        while (motorBackRight.isBusy()) {
        }
        motorBackRight.setPower(0);
        motorBackRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void moveForwards(int distance, double power) {
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackRight.setTargetPosition(-distance);
        motorBackLeft.setTargetPosition(distance);

        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBackRight.setPower(power);
        motorBackLeft.setPower(power);

        while (motorBackLeft.isBusy() && motorBackRight.isBusy()) {
        }

        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void rotateRight(int distance, double power) {
        rotateLeft(-distance, power);
    }

    public void moveBackwards(int distance, double power) {
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackRight.setTargetPosition(distance);
        motorBackLeft.setTargetPosition(-distance);

        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBackRight.setPower(power);
        motorBackLeft.setPower(power);

        while (motorBackLeft.isBusy() && motorBackRight.isBusy()) {
        }

        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void extend(double power, int distance){
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setTargetPosition(distance);
        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extension.setPower(power);
        while (extension.isBusy()) {
        }
        extension.setPower(0);
        extension.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
    public void extend2(double power, int distance) throws InterruptedException{
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setTargetPosition(distance);
        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extension.setPower(power);
        while (extension.isBusy()) {
            lowerLift();
        }
        extension.setPower(0);
        extension.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lowerLift();
    }
    public void parkInCrater(){
        extend(1,500);
    }
    public void parkInCraterCenter(int distance, double power) throws InterruptedException{
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackRight.setTargetPosition(-distance);
        motorBackLeft.setTargetPosition(distance);
        extension.setTargetPosition(500);

        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBackRight.setPower(power);
        motorBackLeft.setPower(power);
        extension.setPower(1);

        while (motorBackRight.isBusy() && motorBackLeft.isBusy() && extension.isBusy()){
            lowerLift2();
        }
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
        extension.setPower(0);
        motorBackRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        extension.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void lowerLift() throws InterruptedException {
        while (!topLimit.isPressed()) {
            lift1.setPower(1);
            lift2.setPower(1);
            if (topLimit.isPressed()) {
                lift1.setPower(0);
                lift2.setPower(0);
            }
        }
    }

    private void stopLift () throws InterruptedException {
        lift1.setPower(0);
        lift2.setPower(0);
        Thread.sleep(500);
    }
    private void intakeOut() throws InterruptedException{
        intake.setPower(-1);
        Thread.sleep(200);
        intake.setPower(0);
    }

    private void intakeIn() throws InterruptedException {
        flipper1.setPosition(0.6);
        flipper2.setPosition(0.4);
        while (!intakeUp) {
            intake.setPower(1);
            Thread.sleep(1000);
            flipperUp();
            if (flipper1.getPosition() == 0.15 && flipper2.getPosition() == 0.85) {
                intakeUp = true;
            }
        }
        intake.setPower(1);
        Thread.sleep(200);
        intake.setPower(0);
    }

    private void intakeIn2() throws InterruptedException{
        intake.setPower(1);
        Thread.sleep(500);
        intake.setPower(0);
    }

    public void retract2(double power, int distance){
        intakeRetract = true;
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setTargetPosition(-distance);
        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extension.setPower(-power);
        intake.setPower(1);
        while (extension.isBusy()) {
            if (intakeRetract = true){
                intake.setPower(1);
            }
        }
        extension.setPower(0);
        intake.setPower(0);
    }

    public void intakeIn1() throws InterruptedException{
        intake.setPower(1);
        Thread.sleep(200);
        intake.setPower(0);
    }

    public void retract(double power, int distance) throws InterruptedException {
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setTargetPosition(-distance);
        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extension.setPower(-power);
        while (extension.isBusy()) {
            negExtensionCounter = extension.getCurrentPosition();
            telemetry.addData("negExtensionTicks", negExtensionCounter);
            telemetry.update();
            if (negExtensionCounter < -300) {
                flipperDown();
            }
        }
        extension.setPower(0);
    }

    public void moveForwardsIntake(int distance, double power) throws InterruptedException{
        intakeNow = true;
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackRight.setTargetPosition(-distance);
        motorBackLeft.setTargetPosition(distance);

        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBackRight.setPower(power);
        motorBackLeft.setPower(power);

        while (motorBackLeft.isBusy() && motorBackRight.isBusy()) {
            if (intakeNow){
                intake.setPower(1);
            }
        }
        intakeNow = false;

        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void moveBackwardsIntake(int distance, double power) throws InterruptedException {
        intakeNow = true;
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackRight.setTargetPosition(distance);
        motorBackLeft.setTargetPosition(-distance);

        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBackRight.setPower(power);
        motorBackLeft.setPower(power);

        while (motorBackLeft.isBusy() && motorBackRight.isBusy()) {
            if (intakeNow) {
                intakeIn2();
                intakeNow = false;
            }
        }

        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void moveForwards2(int distance, double power) throws InterruptedException{
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackRight.setTargetPosition(-distance);
        motorBackLeft.setTargetPosition(distance);
        extension.setTargetPosition(1500);

        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBackRight.setPower(power);
        motorBackLeft.setPower(power);
        extension.setPower(1);

        while (motorBackRight.isBusy() && motorBackLeft.isBusy() && extension.isBusy()){
            lowerLift2();
        }

        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
        extension.setPower(0);
        motorBackRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        extension.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void moveForwards3(int distance, double power) throws InterruptedException{
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackRight.setTargetPosition(-distance);
        motorBackLeft.setTargetPosition(distance);

        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBackRight.setPower(power);
        motorBackLeft.setPower(power);

        while (motorBackRight.isBusy() && motorBackLeft.isBusy()){
            lowerLift();
        }
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void lowerLift2() throws InterruptedException {
        lift1.setPower(1);
        lift2.setPower(1);
        Thread.sleep(1500);
        while (lift) {
            lift1.setPower(0.7);
            lift2.setPower(0.7);
            extensionCounter = extension.getCurrentPosition();
            telemetry.addData("extensionTicks", extensionCounter);
            telemetry.update();
            if (extensionCounter > 200){
                flipperDown3();
            }
            if (topLimit.isPressed()) {
                lift = false;
                if (!lift){
                    lift1.setPower(0);
                    lift2.setPower(0);
                    intake.setPower(-1);
                    Thread.sleep(300);
                    intake.setPower(0);
                }
            }
        }
    }

    public void raiseLift() throws InterruptedException {
        while (raiseLiftVar == true) {
            lift1.setPower(-0.8);
            lift2.setPower(-0.8);
            if (bottomLimit.isPressed()) {
                raiseLiftVar = false;
                if (raiseLiftVar == false){
                    lift1.setPower(1);
                    lift2.setPower(1);
                    Thread.sleep(200);
                    lift1.setPower(0);
                    lift2.setPower(0);
                }
            }
        }
    }

    public void flipperUp(){
        flipper1.setPosition(0.15);
        flipper2.setPosition(0.85);

    }
    public void flipperDown() throws InterruptedException{
        flipper1.setPosition(0.4);
        flipper2.setPosition(0.6);
    }
    public void flipperDown3() throws InterruptedException{
        flipper1.setPosition(0.4);
        flipper2.setPosition(0.6);
        Thread.sleep(400);
    }

    public void flipperDown2() throws InterruptedException{
        flipper1.setPosition(0.3);
        flipper2.setPosition(0.7);
        Thread.sleep(200);
        flipperDown();
    }

    public void liftWithEncoders(double power, int distance){
        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift1.setTargetPosition(-distance);
        lift2.setTargetPosition(-distance);

        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift1.setPower(power);
        lift2.setPower(power);

        while (lift1.isBusy() && lift2.isBusy()){
        }

        lift1.setPower(0);
        lift2.setPower(0);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void halfTurnLeft(int distance, double power){
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackRight.setTargetPosition(-distance);
        motorBackLeft.setTargetPosition(1/2 * distance);

        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBackRight.setPower(power);
        motorBackLeft.setPower(power);

        while (motorBackRight.isBusy() && motorBackLeft.isBusy()) {
        }

        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void moveForwardsCycle(int distance, double power) throws InterruptedException{
        //extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackRight.setTargetPosition(-distance);
        motorBackLeft.setTargetPosition(distance);
        //extension.setTargetPosition(1100);

        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBackRight.setPower(power);
        motorBackLeft.setPower(power);
        //extension.setPower(1);

        while (motorBackRight.isBusy() && motorBackLeft.isBusy()){
            lowerLift3();
        }

        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
        //extension.setPower(0);
        motorBackRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //extension.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    /*public void liftToMid()throws InterruptedException{
        while(!topLimit.isPressed()){
            lift1.setPower(-1);
            lift2.setPower(-1);
            if (topLimit.isPressed()){
                stop = true;
                if (stop){
                    lift1.setPower(0);
                    lift2.setPower(0);
                }
            }
        }
    }*/

    public void lowerLift3() throws InterruptedException {
        lift1.setPower(1);
        lift2.setPower(1);
        Thread.sleep(500);
        while (lift) {
            lift1.setPower(0.8);
            lift2.setPower(0.8);
            if (topLimit.isPressed()) {
                lift = false;
                if (lift){
                    lift1.setPower(0);
                    lift2.setPower(0);
                }
            }
        }
    }
}
