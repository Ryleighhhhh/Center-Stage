
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous (name = "BlueINFrontofLighthouse", group = "Meet 3")
public class BlueInFrontofLighthouse extends LinearOpMode {
    private DcMotor fL = null;
    private DcMotor frontEncoder = null; //fR
    private DcMotor leftEncoder = null; //bL
    private DcMotor rightEncoder = null; //bR
    private DcMotor spinny = null;
    private DcMotor misumi1 = null;
    private Servo RightPiv = null;
    private Servo fork1 = null;
    private Servo fork2 = null;

    private Servo clawL = null;
    private Servo clawR = null;
    private DcMotor wheel = null;
    private Servo wrist = null;

    private ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.85;
    static final double TURN_SPEED = 0.5;
    double diameter = 15.4;
    double arc90 = Math.PI * diameter / 2;
    double arc180 = Math.PI * diameter;
    double armMove(double deg, int timeout) {
        int goal = (int) (deg * COUNTS_PER_MOTOR_REV / deg);
        return deg;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the drive system variables.
        fL = hardwareMap.get(DcMotor.class, "fL");
        frontEncoder = hardwareMap.get(DcMotor.class, "fR");
        leftEncoder = hardwareMap.get(DcMotor.class, "bL");
        rightEncoder = hardwareMap.get(DcMotor.class, "bR");
        spinny = hardwareMap.get(DcMotor.class, "spinny");
        misumi1 = hardwareMap.get(DcMotor.class, "misumi1");
        clawR = hardwareMap.get(Servo.class, "clawR");
        clawL = hardwareMap.get(Servo.class, "clawL");
        RightPiv = hardwareMap.get(Servo.class, "RightPiv");
        wrist = hardwareMap.get(Servo.class, "wrist");
        wheel = hardwareMap.get(DcMotor.class, "wheel");
        fork1 = hardwareMap.get(Servo.class, "fork1");
        fork2 = hardwareMap.get(Servo.class, "fork2");

        fL.setDirection(DcMotor.Direction.REVERSE);
        frontEncoder.setDirection(DcMotor.Direction.FORWARD);
        leftEncoder.setDirection(DcMotor.Direction.REVERSE);
        rightEncoder.setDirection(DcMotor.Direction.FORWARD);
        spinny.setDirection(DcMotor.Direction.FORWARD);
        misumi1.setDirection(DcMotorSimple.Direction.REVERSE);
        clawL.setPosition(0.1);
        clawR.setPosition(0.8);

        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        misumi1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        misumi1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        misumi1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.setMsTransmissionInterval(20);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d : %7d :%7d",
                fL.getCurrentPosition(),
                frontEncoder.getCurrentPosition(),
                leftEncoder.getCurrentPosition(),
                rightEncoder.getCurrentPosition());
        telemetry.update();

        telemetry.addData("Ending at", "%7d :%7d : %7d :%7d",
                fL.getCurrentPosition(),
                frontEncoder.getCurrentPosition(),
                leftEncoder.getCurrentPosition(),
                rightEncoder.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        encoderDrive(DRIVE_SPEED, -5, 5, 5, -5, 5.0); //strafes left to left island
        encoderDrive(DRIVE_SPEED, 5, 5, 5, 5, 5.0); //forward
        RightPiv.setPosition(0);
        wrist.setPosition(0.49);
        clawL.setPosition(0.75);
        encoderDrive(DRIVE_SPEED, -5, -5, -5, -5, 5.0); //forward
        encoderDrive(DRIVE_SPEED, 10, -10, -10, 10, 5.0); //strafes left to left island
        encoderDrive(DRIVE_SPEED, 5, 5, 5, 5, 5.0); //forward
        clawR.setPosition(0.55);


        telemetry.addData("Ending at", "%7d :%7d",
                fL.getCurrentPosition(),
                frontEncoder.getCurrentPosition(),
                leftEncoder.getCurrentPosition(),
                rightEncoder.getCurrentPosition());
        telemetry.update();
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

public void encoderDrive(double speed,
        double leftfrontwheelInches, double rightfrontwheelInches,
        double leftbackwheelInches, double rightbackwheelInches,
        double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

        // Determine new target position, and pass to motor controller
        newFrontLeftTarget = fL.getCurrentPosition() + (int) (leftfrontwheelInches * COUNTS_PER_INCH);
        newFrontRightTarget = frontEncoder.getCurrentPosition() + (int) (rightfrontwheelInches * COUNTS_PER_INCH);
        newBackLeftTarget = leftEncoder.getCurrentPosition() + (int) (leftbackwheelInches * COUNTS_PER_INCH);
        newBackRightTarget = rightEncoder.getCurrentPosition() + (int) (rightbackwheelInches * COUNTS_PER_INCH);
        fL.setTargetPosition(newFrontLeftTarget);
        frontEncoder.setTargetPosition(newFrontRightTarget);
        leftEncoder.setTargetPosition(newBackLeftTarget);
        rightEncoder.setTargetPosition(newBackRightTarget);

        // Turn On RUN_TO_POSITION
        fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontEncoder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftEncoder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightEncoder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        fL.setPower(Math.abs(speed));
        frontEncoder.setPower(Math.abs(speed));
        leftEncoder.setPower(Math.abs(speed));
        rightEncoder.setPower(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (opModeIsActive() &&
        (runtime.seconds() < timeoutS) &&
        (fL.isBusy() && frontEncoder.isBusy() && leftEncoder.isBusy() && rightEncoder.isBusy())) {

        // Display it for the driver.
        telemetry.addData("Running to", " %7d :%7d", newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
        telemetry.addData("Currently at", " at %7d :%7d",
        fL.getCurrentPosition(), frontEncoder.getCurrentPosition(),
        leftEncoder.getCurrentPosition(), rightEncoder.getCurrentPosition());
        telemetry.update();
        }

        fL.setPower(0);
        frontEncoder.setPower(0);
        leftEncoder.setPower(0);
        rightEncoder.setPower(0);

        // Turn off RUN_TO_POSITION
        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(250);   // optional pause after each move.

        }
        }
public void spinDrive(double speed, double spinInches, double timeoutS){
        int newspintarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

        // Determine new target position, and pass to motor controller
        newspintarget = spinny.getCurrentPosition() + (int) (spinInches * COUNTS_PER_INCH);
        spinny.setTargetPosition(newspintarget);

        // Turn On RUN_TO_POSITION
        spinny.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        spinny.setPower(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (opModeIsActive() &&
        (runtime.seconds() < timeoutS) &&
        (spinny.isBusy())) {

        // Display it for the driver.
        telemetry.addData("Running to", "%7d", newspintarget);
        telemetry.addData("Currently at", "at %7d",
        spinny.getCurrentPosition());
        telemetry.update();
        }

        spinny.setPower(0);

        // Turn off RUN_TO_POSITION
        spinny.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(250);   // optional pause after each move.
        }
        }
        }
    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */

