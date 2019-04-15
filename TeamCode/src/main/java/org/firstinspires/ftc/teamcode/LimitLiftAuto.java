package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@Autonomous(name = "LimitLiftAuto", group = "Autonomous")
//Declare motors
public class LimitLiftAuto extends LinearOpMode {
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
    private TouchSensor topLimit;
    private TouchSensor inLimit;
    private DcMotor intake;
    private Servo flipper1;
    private Servo flipper2;
    private ElapsedTime runtime = new ElapsedTime();
    private TouchSensor bottomLimit;
    private double time2 = 1;
    private double time1 = 1;
    private Servo liftServo1;
    private Servo liftServo2;
    double extensionCounter;
    double negExtensionCounter;
    boolean intakeUp = false;
    boolean lift = true;
    private TouchSensor midLimit;
    boolean intakeNow = false;
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
        sampleArm = hardwareMap.servo.get("sampleArm");

        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        strafingRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        strafingLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extension = hardwareMap.dcMotor.get("extension");
        extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topLimit = hardwareMap.touchSensor.get("topLimit");
        inLimit = hardwareMap.touchSensor.get("inLimit");
        intake = hardwareMap.dcMotor.get("intake");
        bottomLimit = hardwareMap.touchSensor.get("bottomLimit");
        flipper1 = hardwareMap.servo.get("flipper1");
        flipper2 = hardwareMap.servo.get("flipper2");
        liftServo1 = hardwareMap.servo.get("liftServo1");
        liftServo2 = hardwareMap.servo.get("liftServo2");
        midLimit = hardwareMap.touchSensor.get("midLimit");


        liftServo1.setPosition(0.96);
        liftServo2.setPosition(0.72);


        //waitForStart();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "waiting for start command...");
            telemetry.update();
        }
        liftToMid();
        /*lift1.setPower(-1);
        lift2.setPower(-1);
        Thread.sleep(1160);*/



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
    public void moveBackwardsIntake(int distance, double power) throws InterruptedException {
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
    private void intakeIn2() throws InterruptedException{
        intake.setPower(1);
        Thread.sleep(500);
        intake.setPower(0);
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

    public void teleLift(){
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
        }
        if (!topLimit.isPressed()) {
            lift1.setPower(0);
            lift2.setPower(0);

        }
    }

    private void intakeIn() throws InterruptedException {
        while (intakeUp == false) {
            intake.setPower(1);
            Thread.sleep(200);
            flipperUp();
            if (flipper1.getPosition() == 0.15 && flipper2.getPosition() == 0.85) {
                intakeUp = true;
            }
        }
        intake.setPower(1);
        Thread.sleep(800);
        intake.setPower(0);
    }
    public void intakeIn1() throws InterruptedException{
        intake.setPower(1);
        Thread.sleep(200);
        intake.setPower(0);
    }
    public void lowerLift2() throws InterruptedException {
        lift1.setPower(1);
        lift2.setPower(1);
        Thread.sleep(1500);
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
    public void lowerLift3() throws InterruptedException {
        lift1.setPower(1);
        lift2.setPower(1);
        Thread.sleep(500);
        while (lift == true) {
            extensionCounter = extension.getCurrentPosition();
            telemetry.addData("extensionTicks", extensionCounter);
            telemetry.update();
            if (extensionCounter > 50){
                flipperDown();
                intakeOut();
            }
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
    public void lowerRobot() {
        while (!topLimit.isPressed()) {
            lift1.setPower(1);
            lift2.setPower(1);
            if (topLimit.isPressed()) {
                lift1.setPower(0);
                lift2.setPower(0);
            }
        }
    }
    public void flipperDown(){
        flipper1.setPosition(0.4);
        flipper2.setPosition(0.6);
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

    public void retract(double power, int distance){
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setTargetPosition(-distance);
        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extension.setPower(-power);
        while (extension.isBusy()){
            negExtensionCounter = extension.getCurrentPosition();
            telemetry.addData("negExtensionTicks", negExtensionCounter);
            telemetry.update();
            if (negExtensionCounter < -500){
                flipper1.setPosition(0.0);
                flipper2.setPosition(1.0);
            }
        }
    }
    public void intakeOut() throws InterruptedException{
        intake.setPower(-1);
        Thread.sleep(200);
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
            lowerRobot();
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
            lowerLift();
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
    public void liftToMid(){
        while(!midLimit.isPressed()){
            lift1.setPower(-1);
            lift2.setPower(-1);
            if (midLimit.isPressed()){
                lift1.setPower(0);
                lift2.setPower(0);
            }
        }
    }
    public void halfTurnLeft(int distance, double power){
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackRight.setTargetPosition(-distance);
        motorBackLeft.setTargetPosition(distance);

        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBackRight.setPower(power);
        motorBackLeft.setPower(2/3* power);

        while (motorBackLeft.isBusy() && motorBackRight.isBusy()) {
        }

        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
    public void flipperUp(){
        flipper1.setPosition(0.15);
        flipper2.setPosition(0.85);

    }
    public void moveForwardsCycle(int distance, double power) throws InterruptedException{
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
            lowerLift3();
        }
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
        extension.setPower(0);
        motorBackRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        extension.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
}

