//Adham don't touch anything besides:
//1. lines 164/165 for lift
//2. lines 129-133 for center sample (this one should be good though)
//3. lines 141 & 144 for sample left (only change values labeled distance:)
//4. lines 151 & 154 for sample right (only change values labeled distance:)
//5. For left sample path = 252-256
//6. For right sample path = 248-252

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.path.LineSegment;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
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

@Autonomous(name = "AutoSample Crater", group = "Autonomous")

//Declare motors
public class AutoSampleCrater extends LinearOpMode {
    private DcMotor lift1;
    private DcMotor lift2;
    private DcMotor strafingRight;
    private DcMotor strafingLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    private Servo leftSampleArm;
    private Servo phoneMount;
    private GoldAlignDetector detector;
    boolean center = false;
    boolean left = false;
    private Servo rightIntakeFlipper;
    private Servo leftIntakeFlipper;
    private DcMotor extension;
    ElapsedTime timer = new ElapsedTime();
    double startTime = timer.time();
    private TouchSensor topLimit;
    private TouchSensor bottomLimit;
    private Servo landerFlipper;



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");

        //Set up detector with phone camera
        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
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
        leftSampleArm = hardwareMap.servo.get("leftSampleArm");
        phoneMount = hardwareMap.servo.get("phoneMount");

        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        strafingRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        strafingLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightIntakeFlipper = hardwareMap.servo.get("rightIntakeFlipper");
        leftIntakeFlipper = hardwareMap.servo.get("leftIntakeFlipper");
        extension = hardwareMap.dcMotor.get("extension");
        extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topLimit = hardwareMap.touchSensor.get("topLimit");
        bottomLimit = hardwareMap.touchSensor.get("bottomLimit");
        landerFlipper = hardwareMap.servo.get("landerFlipper");


        //waitForStart();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "waiting for start command...");
            telemetry.update();
        }
        detector.disable();
        leftIntakeFlipper.setPosition(0.5);
        rightIntakeFlipper.setPosition(0.5);
        landerFlipper.setPosition(0.05);
        leftSampleArm.setPosition(0.4);
        phoneMount.setPosition(0.8);
        Thread.sleep(500);
        detector.enable();
        Thread.sleep(2000);

        //Telemetry returned X-Value for when block is seen in center position
        if (detector.getXPosition() >= 400 && detector.getXPosition() <= 600){
            center = true;
        }

        Thread.sleep(100);
        telemetry.addData("center", center);
        telemetry.update();
        Thread.sleep(100);

        //Telemetry returned X-Value for when block is seen in left position
        if (detector.getXPosition() < 300 && detector.getXPosition() > 10){
            left = true;
        }
        telemetry.addData("left",left);
        telemetry.update();
        Thread.sleep(100);

        detector.disable();
        phoneMount.setPosition(0.43);
        Thread.sleep(500);

        lowerRobot();
        Thread.sleep(100);

        //Code to run if block is seen in center position, if variable center is returned as true
        if(center == true){
            sampleCenter();
            moveToDepot();
            rotateLeft(1200,0.3);
            moveForwards(2000,0.5);
            parkInCrater();
            lowerLift();
        }
        //Code to run if block is seen in left position, if variable left is returned as true
        if(left == true){
            sampleLeft();
            rotateLeft(450,0.5);
            Thread.sleep(200);
            moveForwards(1530,0.5);
            Thread.sleep(200);
            rotateLeftSlow(880,0.5);
            Thread.sleep(200);
            moveForwards(2000,0.5);
            rotateLeft(200,0.2);
            teamMarker();
            rotateRight(200,0.2);
            Thread.sleep(200);
            rotateLeft(1200,0.3);
            moveForwards(2100,0.5);
            parkInCrater();
            lowerLift();
        }
        //Code to run if block is in right position, not visible as an X-Value returned but rather as the condition
        //when both left and center are negated as true conditions
        if(left == false && center == false){
            sampleRight();
            moveToDepot();
            rotateLeft(1200,0.3);
            moveForwards(2000,0.5);
            parkInCrater();
            lowerLift();
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
        leftSampleArm.setPosition(0.9);
        Thread.sleep(1000);
        leftSampleArm.setPosition(0.3);
    }
    public void rotateLeft(int distance, double power){
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackRight.setTargetPosition(distance);
        motorBackLeft.setTargetPosition(distance);

        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBackRight.setPower(1);
        motorBackLeft.setPower(1);
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
    public void rotateRight(int distance, double power){
        rotateLeft(-distance, power);
    }
    public void moveBackwards(int distance, double power){
        moveForwards(-distance, power);
    }
    public void sampleCenter() throws InterruptedException{
        moveForwards(400,0.5);
        Thread.sleep(500);
        //rotateLeft(300,0.5);
        moveForwards(500,0.5);
        Thread.sleep(200);
        moveBackwards(600,0.5);
    }
    public void sampleLeft() throws InterruptedException{
        moveForwards(400,0.5);
        Thread.sleep(500);
        rotateLeft(350,0.5);
        Thread.sleep(400);
        moveForwards(780,0.3);
        Thread.sleep(200);
        moveBackwards(780,0.3);
        rotateRight(360,0.5);
        Thread.sleep(200);
    }
    public void sampleRight() throws InterruptedException{
        moveForwards(400,0.5);
        Thread.sleep(500);
        rotateRight(300,0.5);
        Thread.sleep(400);
        moveForwards(750,0.5);
        Thread.sleep(200);
        moveBackwards(750,0.3);
        rotateLeft(320,0.5);
        Thread.sleep(200);
    }

    public void moveToDepot() throws InterruptedException{
        rotateLeft(450,0.5);
        Thread.sleep(200);
        moveForwards(1400,0.5);
        Thread.sleep(200);
        rotateLeftSlow(880,0.5);
        Thread.sleep(200);
        moveForwards(2000,0.5);
        rotateLeft(200,0.2);
        teamMarker();
        rotateRight(200,0.2);
        Thread.sleep(200);
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
    public void extend(double power, int distance) {
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
    public void lowerLift(){
        while (!topLimit.isPressed()) {
            lift1.setPower(1);
            lift2.setPower(1);
            if (topLimit.isPressed()) {
                lift1.setPower(0);
                lift2.setPower(0);
            }
        }
    }
}