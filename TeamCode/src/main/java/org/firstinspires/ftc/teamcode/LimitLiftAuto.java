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

@Autonomous(name = "LimitLiftAuto", group = "Autonomous")

//Declare motors
public class LimitLiftAuto extends LinearOpMode {
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
    private TouchSensor inLimit;
    private DcMotor intake;


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

        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        strafingRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        strafingLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightIntakeFlipper = hardwareMap.servo.get("rightIntakeFlipper");
        leftIntakeFlipper = hardwareMap.servo.get("leftIntakeFlipper");
        extension = hardwareMap.dcMotor.get("extension");
        extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topLimit = hardwareMap.touchSensor.get("topLimit");
        inLimit = hardwareMap.touchSensor.get("inLimit");
        intake = hardwareMap.dcMotor.get("intake");



        //waitForStart();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "waiting for start command...");
            telemetry.update();
        }
        lowerRobot();
        Thread.sleep(100);
        intakeOut();

        //Code to run if block is seen in center position, if variable center is returned as true

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

    public void retract(){
        while (!inLimit.isPressed()) {
            extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            extension.setPower(-1);
        }
        if (inLimit.isPressed()) {
            extension.setPower(0);
        }
    }
    public void moveForwards3(int distance, double power){
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift1.setTargetPosition(4560);
        lift2.setTargetPosition(4560);
        extension.setTargetPosition(2000);

        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift1.setPower(1);
        lift2.setPower(1);
        extension.setPower(1);

        while (lift1.isBusy() && lift2.isBusy() && extension.isBusy()){
        }
        lift1.setPower(0);
        lift2.setPower(0);
        extension.setPower(0);
    }
    public void intakeIn() throws InterruptedException{
        intake.setPower(1);
        Thread.sleep(200);
        intake.setPower(0);
    }
    public void intakeOut() throws InterruptedException{
        intake.setPower(-1);
        Thread.sleep(200);
        intake.setPower(0);
    }
}

