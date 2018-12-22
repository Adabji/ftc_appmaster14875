package org.firstinspires.ftc.robotcontroller;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.sun.tools.javac.tree.DCTree;

import java.util.concurrent.SynchronousQueue;
@Disabled
@Autonomous (name = "EncoderAuto", group = "Autonomous")
public class EncoderAuto extends LinearOpMode {
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    private DcMotor strafingRight;
    private DcMotor strafingLeft;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        strafingRight = hardwareMap.dcMotor.get("strafingRight");
        strafingLeft = hardwareMap.dcMotor.get("strafingLeft");

        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        strafingRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        strafingLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        DriveForward(1, TICKS_PER_REV);
        Turn(1, TICKS_PER_REV);
        Turn(-1, TICKS_PER_REV);
    }

        int TICKS_PER_REV = 538;
        public void DriveForward (double power, int distance){
            //Reset encoders
            motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //Set target position
            motorBackRight.setTargetPosition(distance);
            motorBackLeft.setTargetPosition(distance);

            //Run to position mode
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Set motor power
            motorBackRight.setPower(power);
            motorBackLeft.setPower(power);

            while(motorBackRight.isBusy() || motorBackLeft.isBusy()){
                //Wait until target position is reached
            }

            //Stop driving && set mode back to RUN_WITH_ENCODERS
            Stop();
            motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        }
        public void Turn (double power, int distance) {
            //Reset encoders
            strafingRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            strafingLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //Set target position
            strafingRight.setTargetPosition(distance);
            strafingLeft.setTargetPosition(distance);

            //Run to position mode
            strafingRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            strafingLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Set motor power
            strafingRight.setPower(power);
            strafingLeft.setPower(power);

            while (strafingRight.isBusy() || strafingLeft.isBusy()) {
                //Wait until target position is reached
            }

            //Stop driving && set mode back to RUN_WITH_ENCODERS
            Stop();
            strafingRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            strafingLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
        //Stop normal motors
        public void Stop () {
            motorBackRight.setPower(0);
            motorBackLeft.setPower(0);
        }
}

