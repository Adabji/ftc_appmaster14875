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
import com.qualcomm.robotcore.hardware.Servo;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.SynchronousQueue;
import java.util.concurrent.ThreadLocalRandom;

@Autonomous(name = "AutoSample Depot", group = "Autonomous")

//Declare motors
public class AutoSampleDepot extends LinearOpMode {
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
    private TouchSensor bottomLimit;



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
        bottomLimit = hardwareMap.touchSensor.get("bottomLimit");


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
        Thread.sleep(1000);
        detector.enable();
        Thread.sleep(1000);

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
        //turnRight(300, 0.3);
        Thread.sleep(100);

        //Code to run if block is seen in center position, if variable center is returned as true
        if(center == true){
//<<<<<<< HEAD
            moveForwards(1500,1);
            rotateLeft(150);
            moveForwards(100,1);
            //turnLeft(270, 0.3);
//=======
            // Knocking off sample
            moveForwards(900,1);
            moveForwards(900,1);
            //Angling towards depot
            rotateLeft(150);
            Thread.sleep(300);
//>>>>>>> 67b1925f3867335fd4fd039715b1413ac66c7637
            teamMarker();
            rotateRight(150);
            moveBackwards(350,1);
            moveBackwards(350,1);
            moveBackwards(350,1);
            Thread.sleep(400);
            rotateLeft(550);
            Thread.sleep(300);
            moveForwards(1600,1);
            Thread.sleep(200);
            rotateLeft(300);
//<<<<<<< HEAD
            Thread.sleep(600);
            moveForwards(400,1);
            Thread.sleep(300);
            rotateLeft(400);
//=======
            Thread.sleep(200);
            turnLeft(1000,.5);
            turnRight(100,1);
            moveForwards(400,1);
            teamMarker();
            Thread.sleep(1000);

//>>>>>>> 67b1925f3867335fd4fd039715b1413ac66c7637
           /* rotateLeft(500);
            moveForwards(400);
            rotateLeft(350);
            moveForwards(2000); */
        }
        //Code to run if block is seen in left position, if variable left is returned as true
        if(left == true){
            moveForwards(300,1);
            sampleLeft();
            teamMarker();
            moveBackwards(100,1);
            rotateLeft(800);
//<<<<<<< HEAD
            moveForwards(1900,1);
//=======
            moveForwards(2200,1);
//>>>>>>> 67b1925f3867335fd4fd039715b1413ac66c7637
            teamMarker();



        }
        //Code to run if block is in right position, not visible as an X-Value returned but rather as the condition
        //when both left and center are negated as true conditions
        if(left == false && center == false){
            moveForwards(500,1);
            sampleRight();
            teamMarker();
            moveBackwards(100,1);
            Thread.sleep(300);
            rotateLeft(300);
            Thread.sleep(300);
            moveForwards(1000,1);
            Thread.sleep(300);
            rotateLeft(300);
            Thread.sleep(300);
            turnLeft(1000,.5);
            turnRight(50,1);
            Thread.sleep(300);
            moveForwards(1600,1);
            teamMarker();
        }
   }


    public void lowerRobot() {
      /*  lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Adham you can change these encoder values if you need the lift to go lower or higher
        //JUST MAKE SURE THEY ARE BOTH NEGATIVE OR ELSE THE LIFT WILL BREAK
        lift1.setTargetPosition(-4560);
        lift2.setTargetPosition(-4560);

        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift1.setPower(1);
        lift2.setPower(1);

        while (lift1.isBusy() || lift2.isBusy()) {
        }

        lift1.setPower(0);
        lift2.setPower(0); */
        while (!bottomLimit.isPressed()) {
            lift1.setPower(-1);
            lift2.setPower(-1);
            if (bottomLimit.isPressed()) {
                lift1.setPower(0);
                lift2.setPower(0);
            }
        }
    }

    public void turnRight(int distance, double power) throws InterruptedException{
        strafingRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafingLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        strafingRight.setTargetPosition(distance);
        strafingLeft.setTargetPosition(distance);

        strafingRight.setPower(power);
        strafingLeft.setPower(power);

        strafingRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        strafingLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (strafingRight.isBusy() || strafingLeft.isBusy()) {
            //Thread.sleep(500);
           /* if (!strafingRight.isBusy()){
                if (!strafingLeft.isBusy()){
                    strafingRight.setPower(0);
                    strafingLeft.setPower(0);
                }
            }*/
        }

        strafingRight.setPower(0);
        strafingLeft.setPower(0);
    }

    public void turnLeft(int distance, double power) throws InterruptedException{
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
//<<<<<<< HEAD
        rotateRight(350);
        moveForwards(1300,1);
        rotateLeft(550);
        moveForwards(1100,1);
//=======
        rotateRight(300);
        Thread.sleep(200);
        moveForwards(1300,1);
        Thread.sleep(200);
        rotateLeft(600);
        Thread.sleep(200);
        moveForwards(900,1);
        Thread.sleep(200);
//>>>>>>> 67b1925f3867335fd4fd039715b1413ac66c7637
    }
    public void sampleLeft() throws InterruptedException {
        rotateLeft(250);
        moveForwards(1150,1);
        rotateRight(250);
        moveForwards(900,1);


    }
    public void teamMarker() throws InterruptedException{
        leftSampleArm.setPosition(0.9);
        Thread.sleep(1000);
        leftSampleArm.setPosition(0.3);
        leftIntakeFlipper.setPosition(0.3);
        rightIntakeFlipper.setPosition(0.7);
    }
    public void rotateLeft(int distance) throws InterruptedException{
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackRight.setTargetPosition(distance);
        motorBackLeft.setTargetPosition(distance);

        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBackRight.setPower(1);
        motorBackLeft.setPower(1);
        while (motorBackRight.isBusy() || motorBackLeft.isBusy()){
           // Thread.sleep(500);
            /*if (!motorBackRight.isBusy()){
                if (!motorBackLeft.isBusy()){
                    motorBackRight.setPower(0);
                    motorBackLeft.setPower(0);
                }
            }*/
        }

        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

    }
    public void moveForwards(int distance, double power) throws InterruptedException{
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackRight.setTargetPosition(-distance);
        motorBackLeft.setTargetPosition(distance);

        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBackRight.setPower(power);
        motorBackLeft.setPower(power);

        while (motorBackRight.isBusy() || motorBackLeft.isBusy()){
           // Thread.sleep(500);
            /*if (!motorBackRight.isBusy()){
                if (!motorBackLeft.isBusy()){
                    motorBackRight.setPower(0);
                    motorBackLeft.setPower(0);
                }
            }*/
        }

        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
    }
    public void rotateRight(int distance) throws InterruptedException{
        rotateLeft(-distance);
    }
    public void moveBackwards(int distance, double power) throws InterruptedException{
        moveForwards(-distance, power);
    }
    public void parkInCrater() throws InterruptedException{
        //This space is for any commands like rotate or move forward (if it stalls add Thread.sleep(time in milliseconds)
        //left arm down
        leftSampleArm.setPosition(0.9);
        Thread.sleep(100);
    }




}

/* */



  //  turnLeft(500,1);
  //  parkInCrater();
