package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name = "Motion Profiling Auto", group = "Autonomous")

public class MotionProfilingAuto extends LinearOpMode {
        private DcMotor strafingRight;
        private DcMotor strafingLeft;
        private DcMotor motorBackRight;
        private DcMotor motorBackLeft;

        @Override
        public void runOpMode() throws InterruptedException {
            motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
            motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");

            strafingRight = hardwareMap.dcMotor.get("strafingRight");
            strafingLeft = hardwareMap.dcMotor.get("strafingLeft");


            while (!opModeIsActive() && !isStopRequested()) {
                telemetry.update();
                telemetry.addData("Status", "waiting for start command...");
            }
            moveForwards(400,0.5);
        }
        public void moveForwards(int distance, double power) throws InterruptedException {
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

            motorBackRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motorBackLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }
    }


