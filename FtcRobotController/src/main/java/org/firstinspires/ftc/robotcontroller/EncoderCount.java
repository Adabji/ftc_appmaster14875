package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;


//TeleOp program for HDrive
@Disabled
@TeleOp(name = "EncoderCount", group = "TeleOp")
public class EncoderCount extends LinearOpMode {

    //Declare motors
    private DcMotor extension;


    @Override
    public void runOpMode() throws InterruptedException {

//Initialize motors
        extension = hardwareMap.dcMotor.get("extension");
        extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



//Wait for the game to start
        while (!opModeIsActive() && !isStopRequested()){
            telemetry.addData("Status", "waiting for start command...");
            telemetry.update();
        }
        while (opModeIsActive()) {
            telemetry.addData("Extension Distance", extension.getCurrentPosition());
            telemetry.update();
        }
    }
}