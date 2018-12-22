package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
@Autonomous(name = "AutonomousRR", group = "Autonomous")

public class AutonomousRR extends LinearOpMode {
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    private DcMotor strafingRight;
    private DcMotor strafingLeft;

@Override
    public void runOpMode() throws InterruptedException {

        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        strafingRight = hardwareMap.dcMotor.get("strafingRight");
        strafingLeft = hardwareMap.dcMotor.get("strafingLeft");

        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();
        Right(STRAFING_POWER,1000);
        stopStrafing();
        Left(DRIVE_POWER,1000);
        stopStrafing();
        //DriveForward(DRIVE_POWER, 200);
        //Stop();
        //DriveDiagonally(STRAFING_POWER);
        //Stop();

    }

    double DRIVE_POWER = 1.0;
    double STRAFING_POWER = -1.0;
    double HALF_STRAFING_POWER = -0.5;
    double HALF_DRIVE = 0.5;


    public void DriveForward(double power, long time) throws InterruptedException {
        motorBackLeft.setPower(power);
        motorBackRight.setPower(power);
        Thread.sleep(time);
    }

    public void Stop() throws InterruptedException{
        motorBackRight.setPower(HALF_DRIVE);
        motorBackLeft.setPower(HALF_DRIVE);
        Thread.sleep(200);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
    }
    public void stopStrafing() throws InterruptedException{
        strafingRight.setPower(0);
        strafingLeft.setPower(0);
    }

    public void StopAll() throws InterruptedException{
        motorBackLeft.setPower(HALF_DRIVE);
        motorBackRight.setPower(HALF_DRIVE);
        strafingRight.setPower(STRAFING_POWER);
        strafingLeft.setPower(STRAFING_POWER);
        Thread.sleep(200);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
        strafingLeft.setPower(0);
        strafingRight.setPower(0);

    }
    public void DriveDiagonally(double power) throws InterruptedException {
        DriveForward(DRIVE_POWER, 100);
        strafingLeft.setPower(power);
        strafingRight.setPower(power);
        Thread.sleep(100);
        StopAll();
    }

    public void Right(double power, long time) throws InterruptedException {
        strafingRight.setPower(power);
        strafingLeft.setPower(power);
        Thread.sleep(time);
    }
    public void Left(double power, long time) throws InterruptedException {
        strafingRight.setPower(power);
        strafingLeft.setPower(power);
        Thread.sleep(time);
    }
    public void QuickStop(){
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
    }
}

