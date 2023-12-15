package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous (name = "BlueLongScoreVision", group = "Vision Auto")
public class BlueLongScoreVision extends LinearOpMode {
    private DcMotor fL = null;
    private DcMotor fR = null;
    private DcMotor bL = null;
    private DcMotor bR = null;
    /*private DcMotor VP = null;
    private DcMotor VP2 = null;*/
    private DcMotor elbow = null;
    private DcMotor elbow2 = null;
    //private Servo droneservo = null;
    private Servo clawL = null;
    private Servo clawR = null;
    private Servo wrist = null;
    OpenCvCamera webcam;
    VisionPipeline pipeline = new VisionPipeline(telemetry);
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
        fR = hardwareMap.get(DcMotor.class, "fR");
        bL = hardwareMap.get(DcMotor.class, "bL");
        bR = hardwareMap.get(DcMotor.class, "bR");
        /*VP = hardwareMap.get(DcMotor.class, "VP"); //left-rev
        VP2 = hardwareMap.get(DcMotor.class, "VP2"); //right */
        elbow = hardwareMap.get(DcMotor.class, "elbow");
        elbow2 = hardwareMap.get(DcMotor.class, "elbow");
        clawL = hardwareMap.get(Servo.class, "clawL"); //rev
        clawR = hardwareMap.get(Servo.class, "clawR");
        wrist = hardwareMap.get(Servo.class, "wrist");
        //droneservo = hardwareMap.get(Servo.class, "droneservo");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                //320px x 340px
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

                /*
                 * Specify the image processing pipeline we wish to invoke upon receipt
                 * of a frame from the camera. Note that switching pipelines on-the-fly
                 * (while a streaming session is in flight) *IS* supported.
                 */

                webcam.setPipeline(pipeline);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("errorCode", errorCode);
            }
        });

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        fL.setDirection(DcMotor.Direction.FORWARD);
        fR.setDirection(DcMotor.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.FORWARD);
        bR.setDirection(DcMotor.Direction.REVERSE);

        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.setMsTransmissionInterval(20);

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

        switch (pipeline.getAnalysis()) {
            case LEFT:
                clawL.setPosition(0.4); //rest
                clawR.setPosition(0); //rest
                encoderDrive(DRIVE_SPEED, 28, 28, 28, 28, 1.0); //forward
                encoderDrive(DRIVE_SPEED, -arc90, arc90, -arc90, arc90, 5.0); //turns right
                encoderDrive(DRIVE_SPEED, 10, 10, 10, 10, 1.0); //forward for room for claw
                clawL.setPosition(0.4); //rest
                clawR.setPosition(0); //rest
                elbowDrive(DRIVE_SPEED, armMove(-60, 1), 1.0); //arm moves back
                clawL.setPosition(0); //lets go of purple pixel on the marker
                clawR.setPosition(0); //rest
                elbowDrive(DRIVE_SPEED, armMove(85, 1), 1.0); //arm moves forward
                encoderDrive(DRIVE_SPEED, 5, 5, 5, 5, 1.0); //forward for room for claw
                encoderDrive(DRIVE_SPEED, arc90, -arc90, arc90, -arc90, 5.0); //turns left
                encoderDrive(DRIVE_SPEED, 24, 24, 24, 24,1.5); //move forward towards gate
                encoderDrive(DRIVE_SPEED, arc90, -arc90, arc90, -arc90, 5.0); //turns left
                encoderDrive(DRIVE_SPEED,98, 98, 98, 98,2.0); //move forward
                encoderDrive(DRIVE_SPEED, -30, 30, 30, -30, 1.5); //strafe left from front towards board
                encoderDrive(DRIVE_SPEED, -arc180, arc180, -arc180, arc180, 5.0); //forward to back
                encoderDrive(DRIVE_SPEED, -5, 5, 5, -5, 1.5); //strafe left to position
                clawR.setPosition(0); //rest
                elbowDrive(DRIVE_SPEED, armMove(-40, 1), 1.0); //move arm 75 degrees to score
                clawR.setPosition(0.4);
                elbowDrive(DRIVE_SPEED, armMove(40, 1), 1.0); //move arm 75 degrees to score
                encoderDrive(DRIVE_SPEED,-25,25,25,-25,5.0); //strafe right and park
                break;
            case CENTER:
                clawL.setPosition(0.4);
                clawR.setPosition(0);
                encoderDrive(DRIVE_SPEED, 18, 18, 18, 18, 1.0); //move forward
                encoderDrive(DRIVE_SPEED, -arc180, arc180, -arc180, arc180, 5.0); //forward to back
                clawL.setPosition(0.4);
                clawR.setPosition(0);
                elbowDrive(DRIVE_SPEED, armMove(-60, 1), 1.0); //arm moves forward
                clawL.setPosition(0); //lets go of purple pixel on the marker
                clawR.setPosition(0);
                elbowDrive(DRIVE_SPEED, armMove(85, 1), 1.0); //arm moves backward
                clawR.setPosition(0);
                encoderDrive(DRIVE_SPEED, -arc180, arc180, -arc180, arc180, 5.0); //forward to back
                clawR.setPosition(0);
                encoderDrive(DRIVE_SPEED, 15, -15, -15, 15, 1.0); //strafes right from FRONT
                encoderDrive(DRIVE_SPEED, 34, 34, 34, 34,1.5); //move forward towards gate
                clawR.setPosition(0); //rest
                encoderDrive(DRIVE_SPEED, arc90, -arc90, arc90, -arc90, 5.0); //turns left
                encoderDrive(DRIVE_SPEED,98, 98, 98, 98,2.0); //move forward
                clawR.setPosition(0); //rest
                encoderDrive(DRIVE_SPEED, -30, 30, 30, -30, 1.5); //strafe left from front towards board
                encoderDrive(DRIVE_SPEED, -arc180, arc180, -arc180, arc180, 5.0); //forward to back
                clawR.setPosition(0); //rest
                elbowDrive(DRIVE_SPEED, armMove(-40, 1), 1.0); //move arm 75 degrees to score
                clawR.setPosition(0.4);
                elbowDrive(DRIVE_SPEED, armMove(40, 1), 1.0); //move arm 75 degrees to score
                encoderDrive(DRIVE_SPEED,-20,20,20,-20,5.0); //strafe right and park
                break;
            case RIGHT:
                clawL.setPosition(0.4); //rest
                clawR.setPosition(0); //rest
                encoderDrive(DRIVE_SPEED, 18, 18, 18, 18, 1.0); //forward
                encoderDrive(DRIVE_SPEED, 15, -15, -15, 15, 1.0); //strafes right from FRONT
                encoderDrive(DRIVE_SPEED, 10, 10, 10, 10, 1.0); //forward
                encoderDrive(DRIVE_SPEED, -arc90, arc90, -arc90, arc90, 5.0); //turns RIGHT
                clawL.setPosition(0.4);
                clawR.setPosition(0);
                elbowDrive(DRIVE_SPEED, armMove(-60, 1), 1.0); //arm moves forward
                clawL.setPosition(0); //lets go of purple pixel on the marker
                clawR.setPosition(0);
                elbowDrive(DRIVE_SPEED, armMove(85, 1), 1.0); //arm moves forward
                encoderDrive(DRIVE_SPEED, arc90, -arc90, arc90, -arc90, 5.0); //turns LEFT
                encoderDrive(DRIVE_SPEED, 24, 24, 24, 24, 1.0); //forward
                encoderDrive(DRIVE_SPEED, arc90, -arc90, arc90, -arc90, 5.0); //turns LEFT
                encoderDrive(DRIVE_SPEED,98, 98, 98, 98,2.0); //move forward
                encoderDrive(DRIVE_SPEED, -30, 30, 30, -30, 1.5); //strafe towards board
                encoderDrive(DRIVE_SPEED, -arc180, arc180, -arc180, arc180, 5.0); //forward to back
                encoderDrive(DRIVE_SPEED,-5,5,5,-5,5.0); //strafe right to position
                clawR.setPosition(0); //rest
                elbowDrive(DRIVE_SPEED, armMove(-40, 1), 1.0); //move arm 75 degrees to score
                clawR.setPosition(0.4);
                elbowDrive(DRIVE_SPEED, armMove(40, 1), 1.0); //move arm 75 degrees to score
                encoderDrive(DRIVE_SPEED,-18,18,18,-18,5.0); //strafe right and park
                break;
        }

        telemetry.addData("Ending at", "%7d :%7d",
                fL.getCurrentPosition(),
                fR.getCurrentPosition(),
                bL.getCurrentPosition(),
                bR.getCurrentPosition());
        telemetry.update();
        telemetry.addData("Path", "Complete");
        telemetry.update();
        webcam.stopStreaming();
        webcam.closeCameraDevice();
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