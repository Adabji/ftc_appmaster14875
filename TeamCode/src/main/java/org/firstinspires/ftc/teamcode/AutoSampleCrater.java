//Adham don't touch anything besides:
//1. lines 164/165 for lift
//2. lines 129-133 for center sample (this one should be good though)
//3. lines 141 & 144 for sample left (only change values labeled distance:)
//4. lines 151 & 154 for sample right (only change values labeled distance:)
//5. For left sample path = 252-256
//6. For right sample path = 248-252

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.ThreadLocalRandom;

@Autonomous(name = "Sample Crater", group = "Autonomous")

//Declare motors
public class AutoSampleCrater extends LinearOpMode {
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
            liftServo1.setPosition(0.96);
            liftServo2.setPosition(0.72);

            //Telemetry returned X-Value for when block is seen in center position
            if (detector.getXPosition() > 500) {
                right = true;
                center = false;
                telemetry.addData("center", center);
                telemetry.addData("right", right);
                telemetry.update();
            }
            if (detector.getXPosition() > 125 && detector.getXPosition() < 225) {
                center = true;
                right = false;
                telemetry.addData("center", center);
                telemetry.addData("right", right);
                telemetry.update();
            } else if (detector.getAligned() == false) {
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
            liftServo1.setPosition(0.96);
            liftServo2.setPosition(0.72);
            sampleArm.setPosition(0.8);
            stopper.setPosition(0.4);
            lowerRobot();

            //Code to run if block is seen in center position, if variable center is returned as true
            if (center == true) {
                sampleCenter();
                scoreSample();
                moveToDepot();
                //parkInCrater();
            }
            //Code to run if block is seen in left position, if variable left is returned as true
            if (right == true) {
                sampleRight();
                scoreSample();
                moveToDepot();
                //parkInCrater();
            }
            //Code to run if block is in right position, not visible as an X-Value returned but rather as the condition
            //when both left and center are negated as true conditions
            if (right == false && center == false) {
                sampleLeft();
                scoreSample();
                moveToDepot();
                //parkInCrater();
            }
        }
    }
    public void lowerRobot() {
        while (!bottomLimit.isPressed()) {
            lift1.setPower(-1);
            lift2.setPower(-1);
            if (bottomLimit.isPressed()) {
                lift1.setPower(0);
                lift2.setPower(0);
            }
        }
    }
    public void turnRight(int distance, double power) {
        strafingRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafingLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        strafingRight.setTargetPosition(distance);
        strafingLeft.setTargetPosition(distance);

        strafingRight.setPower(power);
        strafingLeft.setPower(power);

        strafingRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        strafingLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (strafingRight.isBusy() && strafingLeft.isBusy()) {
        }

        strafingLeft.setPower(0);
        strafingRight.setPower(0);
        strafingRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        strafingLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
    public void turnLeft(int distance, double power) {
        turnRight(-distance, power);
    }
    public void teamMarker() throws InterruptedException{
        sampleArm.setPosition(0.8);
        Thread.sleep(1000);
        sampleArm.setPosition(0.3);
    }
    public void rotateLeft(int distance, double power){
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackRight.setTargetPosition(distance);
        motorBackLeft.setTargetPosition(distance);

        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBackRight.setPower(power);
        motorBackLeft.setPower(power);
        while (motorBackRight.isBusy() && motorBackLeft.isBusy()){
        }
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

    }
    public void moveForwards(int distance, double power){
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackRight.setTargetPosition(-distance);
        motorBackLeft.setTargetPosition(distance);

        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBackRight.setPower(power);
        motorBackLeft.setPower(power);

        while (motorBackRight.isBusy() && motorBackLeft.isBusy()){
        }
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
    //Robot moves forwards and lowers lift at the same time
    public void moveForwards2(int distance, double power) throws InterruptedException{
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
        lowerLift();
    }
    public void rotateRight(int distance, double power){
        rotateLeft(-distance, power);
    }
    public void moveBackwards(int distance, double power){
        moveForwards(-distance, power);
    }
    public void sampleCenter() throws InterruptedException{
        moveForwards(400,0.5);
        Thread.sleep(200);
        rotateLeft(75,1);
        flipper1.setPosition(.4);
        flipper2.setPosition(.6);
        stopper.setPosition(.5);
        intake.setPower(1);
        extend(1,500);
        lowerLift();
        extend(1,-500);
        flipper1.setPosition(.15);
        flipper2.setPosition(.85);
        stopper.setPosition(.85);
        rotateRight(75,1);
        intake.setPower(0);
        Thread.sleep(200);
        //rotateLeft(300,0.5);
        /*moveForwards(500,0.5);
        Thread.sleep(200);
        moveBackwards(600,0.5);
        rotateLeft(50,.5);*/
    }
    public void sampleLeft() throws InterruptedException{
        moveForwards(400,0.5);
        Thread.sleep(500);
        rotateLeft(270,0.5);
        flipper1.setPosition(.4);
        flipper2.setPosition(.6);
        stopper.setPosition(.5);
        intake.setPower(1);
        extend(1,500);
        lowerLift();
        extend(1,-500);
        flipper1.setPosition(.15);
        flipper2.setPosition(.85);
        stopper.setPosition(.85);
        rotateRight(270,0.5);
        Thread.sleep(200);
        /*moveForwards(780,0.3);
        Thread.sleep(200);
        moveBackwards(780,0.3);
        rotateRight(360,0.5);
        Thread.sleep(200);*/
    }
    public void sampleRight() throws InterruptedException{
        moveForwards(400,0.7);
        Thread.sleep(500);
        rotateRight(210,0.7);
        flipper1.setPosition(.4);
        flipper2.setPosition(.6);
        stopper.setPosition(.5);
        intake.setPower(1);
        extend(1,500);
        lowerLift();
        extend(1,-500);
        flipper1.setPosition(.15);
        flipper2.setPosition(.85);
        stopper.setPosition(.85);
        rotateLeft(210,0.7);
        Thread.sleep(200);
        /*moveForwards(750,0.5);
        rotateLeft(200,.5);
        rotateRight(200,.5);
        Thread.sleep(200);
        moveBackwards(750,0.3);
        rotateLeft(200,0.5);
        Thread.sleep(200);*/
    }
    public void liftScore() throws InterruptedException{
       lift1.setPower(-1);
       lift2.setPower(-1);
       Thread.sleep(1500);
       lift1.setPower(0);
       lift2.setPower(0);
    }
    public void scoreSample() throws InterruptedException{
        liftScore();
        moveBackwards(400,.7);
        liftServo1.setPosition(0.0);
        Thread.sleep(300);
        liftServo2.setPosition(0.15);
        Thread.sleep(1500);
        lowerLiftScore();
    }
    public void moveToDepot() throws InterruptedException{
        moveForwards(400,.8);
        rotateRight(530,0.5);
        Thread.sleep(200);
        moveBackwards(1400,1);
        Thread.sleep(200);
        rotateLeft(320,0.5);
        Thread.sleep(200);
        turnRight(800,1);
        turnLeft(100,1);
        rotateRight(50,1);
        moveBackwards(1800,0.5);
        teamMarker();
        moveForwards(1800,1);
    }
    public void rotateLeftSlow(int distance, double power){
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setTargetPosition(distance);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setPower(power);

        while (motorBackLeft.isBusy()){
        }
        motorBackLeft.setPower(0);
        motorBackLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
    public void rotateRightSlow(int distance, double power){
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setTargetPosition(-distance);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setPower(power);
        while (motorBackRight.isBusy()){
        }
        motorBackRight.setPower(0);
        motorBackRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
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
    public void parkInCrater(){
        extend(1,3000);
    }
    public void lowerLift() throws InterruptedException{
        liftServo1.setPosition(0.96);
        Thread.sleep(10);
        liftServo2.setPosition(0.72);
        lift1.setPower(1);
        lift2.setPower(1);
        Thread.sleep(1000);
        while (!topLimit.isPressed()) {
            lift1.setPower(.5);
            lift2.setPower(.5);
            if (topLimit.isPressed()) {
                lift1.setPower(0);
                lift2.setPower(0);


            }
        }
        lift1.setPower(-1);
        lift2.setPower(-1);
        Thread.sleep(100);
        lift1.setPower(0);
        lift2.setPower(0);
    }
    public void lowerLiftScore() throws InterruptedException{
        liftServo1.setPosition(0.96);
        Thread.sleep(700);
        liftServo2.setPosition(0.72);
        Thread.sleep(500);
        while (!topLimit.isPressed()) {
            lift1.setPower(.8);
            lift2.setPower(.8);
            if (topLimit.isPressed()) {
                lift1.setPower(0);
                lift2.setPower(0);


            }
        }
        lift1.setPower(-1);
        lift2.setPower(-1);
        Thread.sleep(100);
        lift1.setPower(0);
        lift2.setPower(0);
        extend(1,250);
    }
    private void stopLift () throws InterruptedException {
        lift1.setPower(0);
        lift2.setPower(0);
        Thread.sleep(500);
    }

    public void intakeIn() throws InterruptedException{
        intake.setPower(-1);
        Thread.sleep(100);
        intake.setPower(0);
    }
    public void intakeOut() throws InterruptedException{
        intake.setPower(1);
        Thread.sleep(500);
        intake.setPower(-1);
        Thread.sleep(500);
        intake.setPower(0);
    }
    public void extend2(double power, int distance) throws InterruptedException {
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setTargetPosition(distance);
        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extension.setPower(power);
        while (extension.isBusy()) {
            intakeIn();
        }
        extension.setPower(0);
        extension.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lowerLift();
    }
}