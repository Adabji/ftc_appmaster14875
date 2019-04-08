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
    private DcMotor lift1;
    private DcMotor lift2;
    private DcMotor strafingRight;
    private DcMotor strafingLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    private Servo sampleArm;
    private GoldAlignDetector detector;
    boolean center = false;
    boolean left = false;
    private DcMotor extension;
    ElapsedTime timer = new ElapsedTime();
    double startTime = timer.time();
    private TouchSensor bottomLimit;
    private TouchSensor topLimit;
    private DcMotor intake;
    private Servo liftServo1;
    private Servo liftServo2;
    private Servo flipper1;
    private Servo flipper2;
    boolean right = false;
    private TouchSensor inLimit;
    double extensionCounter;
    double negExtensionCounter;
    double intakePower = 0;
    boolean intakeNow = false;
    double time1 = 1;
    double time2 = 1;
    boolean lift = true;
    boolean intakeUp;
    private Servo stopper;
    boolean raiseLiftVar = true;
    boolean sample = false;

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
        inLimit = hardwareMap.touchSensor.get("inLimit");
        intake = hardwareMap.dcMotor.get("intake");
        liftServo1 = hardwareMap.servo.get("liftServo1");

        liftServo2 = hardwareMap.servo.get("liftServo2");


        flipper1 = hardwareMap.servo.get("flipper1");
        flipper2 = hardwareMap.servo.get("flipper2");
        stopper = hardwareMap.servo.get("stopper");
        telemetry.setAutoClear(true);
        detector.enable();

        //waitForStart();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "waiting for start command...");
            flipper1.setPosition(0.12);
            flipper2.setPosition(0.88);
            liftServo1.setPosition(0.96);
            liftServo2.setPosition(0.72);

            //Telemetry returned X-Value for when block is seen in center position
            if (detector.getXPosition() < 100 && detector.getXPosition() > 0) {
                left = true;
                center = false;
                telemetry.addData("center", center);
                telemetry.addData("left", left);
                telemetry.update();
            }
            if (detector.getXPosition() > 400 && detector.getXPosition() < 600) {
                center = true;
                left = false;
                telemetry.addData("center", center);
                telemetry.addData("left", left);
                telemetry.update();
            }
            if (detector.getAligned() == false) {
                left = false;
                center = false;
                telemetry.addData("center", center);
                telemetry.addData("left", left);
                telemetry.update();
            }
            /*if (isStopRequested()) {
                requestOpModeStop();
            }*/


            telemetry.addData("center", center);
            telemetry.addData("left", left);
            telemetry.update();

        }
        if (opModeIsActive()) {
            detector.disable();
            liftServo1.setPosition(0.96);
            liftServo2.setPosition(0.72);
            sampleArm.setPosition(0.8);
            stopper.setPosition(0.7);
            lowerRobot();

            //Code to run if block is seen in center position, if variable center is returned as true
            if (center == true) {
                sampleCenter();
                moveForwardsIntake(400, 1);
                intakeIn();
                retract2(1, 230);
                stopper.setPosition(0.4);
                moveBackwards(400, 1);
                teleLift();
                if (sample == false) {
                    lift1.setPower(-1);
                    lift2.setPower(-1);
                    Thread.sleep(300);
                    sample = true;
                }else {
                    lift1.setPower(0);
                    lift2.setPower(0);
                }
                liftServo1.setPosition(0);
                liftServo2.setPosition(0.3);
                Thread.sleep(2000);
                liftServo1.setPosition(0.96);
                liftServo2.setPosition(0.72);
                rotateLeftSlow(1500,1);
                lift = true;
                moveForwardsCycle(1300,1);
                flipperDown();
                moveForwardsIntake(200,1);



            /*rotateLeft(440, 0.5);
            Thread.sleep(300);
            moveForwards(1100, .5);
            Thread.sleep(200);
            rotateLeftSlow(650, .5);
            Thread.sleep(200);
            parkInCraterCenter(300,1);
            Thread.sleep(500);*/

            }
            //Code to run if block is seen in left position, if variable left is returned as true
            if (left == true) {
                sampleLeft();
                rotateLeft(220, 0.2);
                Thread.sleep(300);
                moveForwardsIntake(430, 0.5);
                intakeIn();
                retract2(1, 250);
                moveBackwards(530, 1);
                Thread.sleep(200);
                rotateRight(210, 0.5);
                teleLift();
                liftWithEncoders(1, 2600);
                liftServo1.setPosition(0);
                liftServo2.setPosition(0.3);
                Thread.sleep(2000);
                liftServo1.setPosition(0.96);
                liftServo2.setPosition(0.72);
                rotateLeftSlow(1280,1);
                lift = true;
                moveForwardsCycle(1300,1);
                flipperDown();
                moveForwardsIntake(200,1);

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
            if (left == false & center == false) {
                sampleRight();
                rotateRight(210, 0.2);
                Thread.sleep(300);
                moveForwardsIntake(400, 0.5);
                intakeIn();
                retract2(1, 250);
                moveBackwards(500, 1);
                Thread.sleep(200);
                rotateLeft(250, 0.5);
                teleLift();
                liftWithEncoders(1, 2600);
                liftServo1.setPosition(0);
                liftServo2.setPosition(0.3);
                Thread.sleep(2000);
                liftServo1.setPosition(0.96);
                liftServo2.setPosition(0.72);
                rotateLeftSlow(1280,1);
                lift = true;
                moveForwardsCycle(1100,1);
                flipperDown();
                moveForwardsIntake(200,1);

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
            lift1.setPower(-1);
            lift2.setPower(-1);
            Thread.sleep(1700);
            while (!bottomLimit.isPressed()) {
                lift1.setPower(-0.8);
                lift2.setPower(-0.8);
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
            moveForwards2(850, .5);
            flipperUp();
            moveBackwards(450, 1);
            retract(1, 800);
            Thread.sleep(400);
        }

        public void sampleLeft () throws InterruptedException {
            moveForwards2(850, .5);
            flipperUp();
            moveBackwards(450, 1);
            retract(1, 800);
            Thread.sleep(400);
        }

        public void sampleCenter () throws InterruptedException {
            moveForwards2(850, .5);
            flipperUp();
            moveBackwards(450, 1);
            retract(1, 800);
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
        flipper1.setPosition(0.5);
        flipper2.setPosition(0.5);
        while (intakeUp == false) {
            intake.setPower(1);
            Thread.sleep(200);
            flipperUp();
            if (flipper1.getPosition() == 0.15 && flipper2.getPosition() == 0.85) {
                intakeUp = true;
            }
        }
        intake.setPower(1);
        Thread.sleep(1500);
        intake.setPower(0);
        intakeUp = false;
    }

    public void retract2(double power, int distance){
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setTargetPosition(-distance);
        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extension.setPower(-power);
        while (extension.isBusy()) {
            }
        extension.setPower(0);
    }
    public void intakeIn1() throws InterruptedException{
        intake.setPower(1);
        Thread.sleep(200);
        intake.setPower(0);
    }
    public void retract(double power, int distance) {
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
        extension.setTargetPosition(1000);

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
        while (lift == true) {
            lift1.setPower(0.4);
            lift2.setPower(0.4);
            extensionCounter = extension.getCurrentPosition();
            telemetry.addData("extensionTicks", extensionCounter);
            telemetry.update();
            if (extensionCounter > 200){
                flipperDown();
                intakeOut();
            }
            if (topLimit.isPressed()) {
                lift = false;
                if (lift == false){
                    intake.setPower(0);
                    lift1.setPower(0);

                    lift2.setPower(0);
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
    public void flipperDown(){
        flipper1.setPosition(0.4);
        flipper2.setPosition(0.6);
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
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackRight.setTargetPosition(-distance);
        motorBackLeft.setTargetPosition(distance);
        extension.setTargetPosition(800);

        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBackRight.setPower(power);
        motorBackLeft.setPower(power);
        extension.setPower(1);

        while (motorBackRight.isBusy() && motorBackLeft.isBusy() && extension.isBusy()){
            lowerLift3();
        }
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
        extension.setPower(0);
        motorBackRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        extension.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
    public void lowerLift3() throws InterruptedException {
        lift1.setPower(1);
        lift2.setPower(1);
        Thread.sleep(500);
        while (lift == true) {
            lift1.setPower(0.4);
            lift2.setPower(0.4);
            if (topLimit.isPressed()) {
                lift = false;
                if (lift == false){
                    intake.setPower(0);
                    lift1.setPower(0);

                    lift2.setPower(0);
                }
            }
        }
    }


}
