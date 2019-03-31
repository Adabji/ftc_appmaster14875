package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;
import java.util.TimerTask;

@TeleOp(name = "Parade Bot Code", group = "TeleOp")
@Disabled
public class ParadeBot extends LinearOpMode {

    //Declare motors
    private DcMotor motorRight, motorLeft, motorTurn;

    @Override
    public void runOpMode() throws InterruptedException {

//Initialize motors
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorTurn = hardwareMap.dcMotor.get("motorTurn");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);


//Wait for the game to start
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "waiting for start command...");
            telemetry.update();
        }
        while (opModeIsActive()) {
            motorLeft.setPower(-gamepad1.left_stick_y);
            motorRight.setPower(-gamepad1.right_stick_y);
            if (gamepad1.right_stick_x > 0.5){
                motorTurn.setPower(1);
            }else if (gamepad1.left_stick_x < -0.5){
                motorTurn.setPower(-1);
            }else {
                motorTurn.setPower(0);
            }
        }
    }
}
