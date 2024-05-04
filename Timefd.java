package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Timefd extends LinearOpMode {

    DcMotor fL;
    DcMotor frontEncoder; //fR
    DcMotor leftEncoder; //bL
    DcMotor rightEncoder; //bR
    DcMotor spinny;
    DcMotor misumi1;
    Servo RightPiv;
    Servo fork1;
    Servo fork2;
    Servo clawL;
    Servo clawR;
    DcMotor wheel;
    Servo wrist;

    @Override
    public void runOpMode() throws InterruptedException {
        fL = hardwareMap.dcMotor.get("fL");
        frontEncoder = hardwareMap.dcMotor.get("fR");
        leftEncoder = hardwareMap.dcMotor.get("bL");
        rightEncoder = hardwareMap.dcMotor.get("bR");
        spinny = hardwareMap.dcMotor.get("spinny");
        misumi1 = hardwareMap.dcMotor.get("misumi1");
        RightPiv = hardwareMap.servo.get("RightPiv");
        fork1 = hardwareMap.servo.get("fork1");
        fork2 = hardwareMap.servo.get("fork2");
        clawL = hardwareMap.servo.get("clawL");
        clawR = hardwareMap.servo.get("clawR");
        wheel = hardwareMap.dcMotor.get("wheel");
        wrist = hardwareMap.servo.get("wrist");

        waitForStart();

        Strafeleft();
        sleep(2000);
        forward();
        sleep(2000);
        RightPiv.setPosition(0);
        wrist.setPosition(0.49);
        clawL.setPosition(0.75);
        sleep(3000);
        Reverse();
        sleep(2000);
        Straferight();
        sleep(2000);
        forward();
        clawR.setPosition(0.55);
        sleep(3000);
    }

    public void forward() {
        fL.setPower(-.7);
        frontEncoder.setPower(.7);
        leftEncoder.setPower(-.7);
        rightEncoder.setPower(.7);
    }

    public void Reverse() {
        fL.setPower(.7);
        frontEncoder.setPower(-.7);
        leftEncoder.setPower(.7);
        rightEncoder.setPower(-.7);
    }

    public void Strafeleft () {
        fL.setPower(-.7);
        frontEncoder.setPower(.7);
        leftEncoder.setPower(.7);
        rightEncoder.setPower(-.7);
    }

    public void Straferight () {
        fL.setPower(.7);
        frontEncoder.setPower(-.7);
        leftEncoder.setPower(-.7);
        rightEncoder.setPower(.7);
    }
}
