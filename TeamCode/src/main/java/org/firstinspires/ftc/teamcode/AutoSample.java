package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.ThreadLocalRandom;

@Autonomous(name = "AutoSample", group = "Autonomous")

//Declare motors
public class AutoSample extends LinearOpMode {
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


        //waitForStart();
        while (!opModeIsActive() && !isStopRequested()){
            telemetry.addData("Status", "waiting for start command...");
            telemetry.update();
        }

        detector.disable();
        leftIntakeFlipper.setPosition(0.4);
        rightIntakeFlipper.setPosition(0.6);
        leftSampleArm.setPosition(0.3);
        phoneMount.setPosition(0.8);
        Thread.sleep(500);
        detector.enable();
        Thread.sleep(2000);

        //Telemetry returned X-Value for when block is seen in center position
        if (detector.getXPosition() >= 400 && detector.getXPosition() <= 550){
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
        turnRight(300, 0.3);
        Thread.sleep(100);

        //Code to run if block is seen in center position, if variable center is returned as true
        if(center == true){
            moveForwards(400);
            turnLeft(270, 0.3);
            moveForwards(1300);
            teamMarker();
            moveForwards(-100);
           /* rotateLeft(500);
            moveForwards(400);
            rotateLeft(350);
            moveForwards(2000); */
        }
        //Code to run if block is seen in left position, if variable left is returned as true
        if(left == true){
            moveForwards(300);
            sampleLeft();
            teamMarker();
            moveForwards(-100);


        }
        //Code to run if block is in right position, not visible as an X-Value returned but rather as the condition
        //when both left and center are negated as true conditions
        if(left == false && center == false){
            moveForwards(300);
            sampleRight();
            teamMarker();
            Thread.sleep(1000);
        }
   }


    public void lowerRobot() {
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift1.setTargetPosition(-4560);
        lift2.setTargetPosition(-4560);

        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift1.setPower(1);
        lift2.setPower(1);

        while (lift1.isBusy() || lift2.isBusy()) {
        }

        lift1.setPower(0);
        lift2.setPower(0);
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

        while (strafingRight.isBusy() || strafingLeft.isBusy()) {
        }

        strafingRight.setPower(0);
        strafingLeft.setPower(0);
    }

    public void turnLeft(int distance, double power) {
        turnRight(-distance, power);
    }
    //Moves the robot to the position where it can rotate to either left/right samples
    //Move to Sample function takes distance values that are altered for center, right
    //and left conditions of the mineral
    public void moveToSample(int distance) {
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackRight.setTargetPosition(-distance);
        motorBackLeft.setTargetPosition(distance);

        lift1.setTargetPosition(4560);
        lift2.setTargetPosition(4560);

        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift1.setPower(1);
        lift2.setPower(1);

        motorBackRight.setPower(0.5);
        motorBackLeft.setPower(0.5);

        while (motorBackRight.isBusy() || motorBackLeft.isBusy()) {
        }
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);

        while (lift1.isBusy() || lift2.isBusy()) {
        }
        lift1.setPower(0);
        lift2.setPower(0);

    }

    public void sampleRight() throws InterruptedException {
        rotateRight(350);
        moveForwards(1300);
        rotateLeft(550);
        moveForwards(1100);
    }
    public void sampleLeft() throws InterruptedException {
        rotateLeft(200);
        moveForwards(1100);
        rotateRight(400);
        moveForwards(800);


    }
    public void teamMarker() throws InterruptedException{
        leftSampleArm.setPosition(0.9);
        Thread.sleep(1000);
        leftSampleArm.setPosition(0.3);
        leftIntakeFlipper.setPosition(0.3);
        rightIntakeFlipper.setPosition(0.7);
    }
    public void rotateLeft(int distance){
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackRight.setTargetPosition(distance);
        motorBackLeft.setTargetPosition(distance);

        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBackRight.setPower(1);
        motorBackLeft.setPower(1);
        while (motorBackRight.isBusy() || motorBackLeft.isBusy()){
        }

        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

    }
    public void moveForwards(int distance){
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackRight.setTargetPosition(-distance);
        motorBackLeft.setTargetPosition(distance);

        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBackRight.setPower(1);
        motorBackLeft.setPower(1);

        while (motorBackRight.isBusy() || motorBackLeft.isBusy()){
        }

        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
    }
    public void rotateRight(int distance){
        rotateLeft(-distance);
    }
    public void parkInCrater() throws InterruptedException{
        leftSampleArm.setPosition(0.8);
        Thread.sleep(100);
    }




}

/* */



  //  turnLeft(500,1);
  //  parkInCrater();
