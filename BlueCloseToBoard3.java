package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous (name = "BlueCloseToBoard3", group = "Meet 3")
public class BlueCloseToBoard3 extends LinearOpMode {
    private DcMotor fL = null;
    private DcMotor fR = null;
    private DcMotor bL = null;
    private DcMotor bR = null;
    private DcMotor elbow = null;
    private DcMotor elbow2 = null;
    private Servo clawL = null;
    private Servo clawR = null;
    private CRServo wrist = null;
    private ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.95;
    static final double TURN_SPEED = 0.5;
    double diameter = 15.4;
    double arc90 = Math.PI * diameter / 2;

    double armMove(double deg, int timeout) {
        int goal = (int) (deg * COUNTS_PER_MOTOR_REV / deg);
        return deg;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the drive system variables.
        fL = hardwareMap.get(DcMotor.class, "fL");
        fR = hardwareMap.get(DcMotor.class, "fR");
        bL = hardwareMap.get(DcMotor.class, "bL");
        bR = hardwareMap.get(DcMotor.class, "bR");
        elbow = hardwareMap.get(DcMotor.class, "elbow");
        elbow2 = hardwareMap.get(DcMotor.class, "elbow2");
        clawL = hardwareMap.get(Servo.class, "clawL");
        clawR = hardwareMap.get(Servo.class, "clawR");
        wrist = hardwareMap.get(CRServo.class, "wrist");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        fL.setDirection(DcMotor.Direction.REVERSE);
        fR.setDirection(DcMotor.Direction.FORWARD);
        bL.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.FORWARD);
        elbow.setDirection(DcMotorSimple.Direction.REVERSE);
        elbow2.setDirection(DcMotorSimple.Direction.FORWARD);

        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d : %7d :%7d",
                fL.getCurrentPosition(),
                fR.getCurrentPosition(),
                bL.getCurrentPosition(),
                bR.getCurrentPosition());
        telemetry.update();

        telemetry.addData("Ending at", "%7d :%7d : %7d :%7d",
                fL.getCurrentPosition(),
                fR.getCurrentPosition(),
                bL.getCurrentPosition(),
                bR.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        clawL.setPosition(0.4);
        clawR.setPosition(0);
        encoderDrive(DRIVE_SPEED, -28, 28, 28, -28, 1.0); //strafes left 28 inches
        //place pixel on marker
        encoderDrive(DRIVE_SPEED, 37, 37, 37, 37, 1.0); //move towards the board
        elbowDrive(DRIVE_SPEED, armMove(-25, 1), 1.0);
        clawL.setPosition(0);
        clawR.setPosition(0.6);
        elbowDrive(DRIVE_SPEED, armMove(-45, 1), 1.0);
        encoderDrive(DRIVE_SPEED, -20, 20, 20, -20, 1.0); //goes forward 20 inches

        telemetry.addData("Ending at", "%7d :%7d",
                fL.getCurrentPosition(),
                fR.getCurrentPosition(),
                bL.getCurrentPosition(),
                bR.getCurrentPosition());
        telemetry.update();

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */

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
            newFrontRightTarget = fR.getCurrentPosition() + (int) (rightfrontwheelInches * COUNTS_PER_INCH);
            newBackLeftTarget = bL.getCurrentPosition() + (int) (leftbackwheelInches * COUNTS_PER_INCH);
            newBackRightTarget = bR.getCurrentPosition() + (int) (rightbackwheelInches * COUNTS_PER_INCH);
            fL.setTargetPosition(newFrontLeftTarget);
            fR.setTargetPosition(newFrontRightTarget);
            bL.setTargetPosition(newBackLeftTarget);
            bR.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            fL.setPower(Math.abs(speed));
            fR.setPower(Math.abs(speed));
            bL.setPower(Math.abs(speed));
            bR.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            //hi ryleigh!! :3

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (fL.isBusy() && fR.isBusy() && bL.isBusy() && bR.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        fL.getCurrentPosition(), fR.getCurrentPosition(),
                        bL.getCurrentPosition(), bR.getCurrentPosition());
                telemetry.update();
            }

            fL.setPower(0);
            fR.setPower(0);
            bL.setPower(0);
            bR.setPower(0);

            // Turn off RUN_TO_POSITION
            fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

    public void elbowDrive(double speed, double elbowInches, double timeoutS) {
        int newelbowtarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newelbowtarget = elbow.getCurrentPosition() + elbow2.getCurrentPosition() + (int) (elbowInches * COUNTS_PER_INCH);
            elbow.setTargetPosition(newelbowtarget);
            elbow2.setTargetPosition(newelbowtarget);

            // Turn On RUN_TO_POSITION
            elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbow2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            elbow.setPower(Math.abs(speed));
            elbow2.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (elbow.isBusy() && elbow2.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", "%7d", newelbowtarget);
                telemetry.addData("Currently at", "at %7d",
                        elbow.getCurrentPosition(), elbow2.getCurrentPosition());
                telemetry.update();
            }

            elbow.setPower(0);
            elbow2.setPower(0);

            // Turn off RUN_TO_POSITION
            elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elbow2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
}