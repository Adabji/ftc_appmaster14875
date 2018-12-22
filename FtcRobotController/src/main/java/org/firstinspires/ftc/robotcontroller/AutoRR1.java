package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.SynchronousQueue;

@Autonomous(name = "AutoRR1", group = "Autonomous")
@Disabled
public class AutoRR1 extends LinearOpMode {
    private DcMotor lift1;
    private DcMotor lift2;
    private DcMotor strafingRight;
    private DcMotor strafingLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    private Servo leftArm;
    private Servo rightArm;


    @Override
    public void runOpMode() throws InterruptedException{
        lift1 = hardwareMap.dcMotor.get("lift1");
        lift2 = hardwareMap.dcMotor.get("lift2");
        strafingRight = hardwareMap.dcMotor.get("strafingRight");
        strafingLeft = hardwareMap.dcMotor.get("strafingLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        leftArm = hardwareMap.servo.get("leftArm");
        rightArm = hardwareMap.servo.get("rightArm");

        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        strafingRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        strafingLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        lowerRobot();
        turnRight (150);
        moveToSample();
        sampleCenter();



    }
    public void lowerRobot(){
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift1.setTargetPosition(-4000);
        lift2.setTargetPosition(-4000);

        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift1.setPower(1);
        lift2.setPower(1);

        while (lift1.isBusy() || lift2.isBusy()) {
        }
        lift1.setPower(0);
        lift2.setPower(0);
    }
    public void turnRight (int distance){
        strafingRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafingLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        strafingRight.setTargetPosition(distance);
        strafingLeft.setTargetPosition(distance);

        strafingRight.setPower(1);
        strafingLeft.setPower(1);

        strafingRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        strafingLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (strafingRight.isBusy() || strafingLeft.isBusy()){
        }

        strafingRight.setPower(0);
        strafingLeft.setPower(0);
    }
    public void turnLeft(int distance){
        turnRight(- distance);
    }
    public void moveToSample(){
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackRight.setTargetPosition(-700);
        motorBackLeft.setTargetPosition(700);

        lift1.setTargetPosition(4000);
        lift2.setTargetPosition(4000);

        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift1.setPower(1);
        lift2.setPower(1);

        motorBackRight.setPower(0.5);
        motorBackLeft.setPower(0.5);

        while (lift1.isBusy() || lift2.isBusy()) {
        }
        lift1.setPower(0);
        lift2.setPower(0);

        while (motorBackRight.isBusy() || motorBackLeft.isBusy()) {
        }
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);

    }
    public void sampleRight() throws InterruptedException{
        leftArm.setPosition(0.63);
        turnLeft (850);
        leftArm.setPosition(0.01);
        Thread.sleep(1000);
    }
    public void sampleCenter() throws InterruptedException{
        rightArm.setPosition(0.24);
        Thread.sleep(200);
        turnLeft(950);
        rightArm.setPosition(0.86);
        Thread.sleep(1000);
    }
    public void sampleLeft() throws InterruptedException {
        rightArm.setPosition(0.24);
        Thread.sleep(200);
        turnRight(850);
        rightArm.setPosition(0.86);
        Thread.sleep(1000);
    }

}


//rightArm.setPosition(0.24);
//rightArm.setPosition(0.86);



