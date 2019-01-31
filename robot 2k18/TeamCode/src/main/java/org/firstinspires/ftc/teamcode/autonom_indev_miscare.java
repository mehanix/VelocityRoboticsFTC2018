/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.HardwareMap;
import org.firstinspires.ftc.teamcode.vision.MasterVision;
import org.firstinspires.ftc.teamcode.vision.SampleRandomizedPositions;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="autonom_indev_miscare", group="Pushbot")
public class autonom_indev_miscare extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareMap robot  = new HardwareMap();   //HardwareMap al robotului nostru
    MasterVision vision;
    SampleRandomizedPositions goldPosition;
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     error_constant = 0.508;
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // encoderul nostru pe AndyMark NeverRest
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION * error_constant) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    static final long     time_per_motor = 500;
    static final long     lift_time = 1500; //1850
    static final long     strafe_time = 500;
    static final long     ramp_time = 245;
    static final long     arm_time = 600;
    static final long     vision_time = 1500;
    static final long     debug_time = 2000;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;// recommended camera direction
        parameters.vuforiaLicenseKey = "AdCqxpv/////AAAAmRYY0W12w0kCiohldJxGqHFZ2GletPTfhXtZhESwlq9pxkGaOfOwJah+6BGckYD4jJ5cmiwelBvWWK7mdEDxpkxkAFJmyxqYcUymQ8BMWBSIsoxbJIWwJ6XeYIRecso9jVV3iF1hgUPJ47uYiB/N7GuqM3DnD+uogQpHCMsy+KS6l0yJSGSTdlGpUu3lwxmVKpjc0Ox0biDe5VdmWNrJJ6DoIuy4TCzectZfidTbMJXwfigDK7dxNLicETcE0RrHgpSQ3F2YWn/ZOoyNZk2JmkPzu4EKPSHIWQfOiIU3KPHP9cjqh7x3NHuvvdyIYJjNb6xpzoA29YQK7l4zNRma0dQSBTyuiSdfkjH/6QSuqiBR";
        //din vazut doar 2, dat seama ce e al 3lea :)
        vision = new MasterVision(parameters, hardwareMap, true, MasterVision.TFLiteAlgorithm.INFER_LEFT);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Incepem la %7d :%7d (roti din fata)",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        //encoderDrive(DRIVE_SPEED,  4,  4, 5.0);
        //encoderDrive(DRIVE_SPEED,  10,  10, 5.0);  // S1: Forward 10 Inches with 5 Sec timeout
        //encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        //encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        //-------------THE PROGRAM ITSELF-------------
        //encoderDrive(TURN_SPEED,   12, -12, 4.0);
        robot.extensionMotorBack.setPower(1);
        sleep(arm_time);
        robot.extensionMotorBack.setPower(0);
        //Brat Rampa Extinde
        robot.lift.setPower(-1);
        sleep(time_per_motor);
        robot.liftBrake.setPosition(robot.liftBrake_unlocked);
        sleep(time_per_motor);
        //Scoate piedica
        robot.lift.setPower(0.5); //0.35
        sleep(lift_time);
        robot.lift.setPower(0);
        sleep(time_per_motor);
        //Coboara robotul
        robot.smallArm.setPosition(robot.smallArm_extended);
        sleep(time_per_motor);
        //Prinde Marker-ul(de sub Lander)
        robot.mecanumDrive_Cartesian(0.3d,0);
        sleep(strafe_time);
        robot.mecanumDrive_Cartesian(0,0);
        sleep(time_per_motor);
        //Iese de pe carlig
        robot.lift.setPower(-0.5); //0.35
        sleep(lift_time);
        robot.lift.setPower(0);
        sleep(time_per_motor);
        //Coboara Liftul
        robot.extensionMotorFront.setPower(robot.extensionMotorFront_forwardSpeed);
        sleep(ramp_time);
        robot.extensionMotorFront.setPower(0);
        robot.extensionMotorBack.setPower(-1);
        sleep(arm_time);
        robot.extensionMotorBack.setPower(0);
        robot.extensionMotorFront.setPower(robot.extensionMotorFront_backSpeed);
        sleep(ramp_time);
        robot.extensionMotorFront.setPower(0);
        //Brat Rampa Contracta
        encoderDrive(TURN_SPEED,   3.3, -3.3, 4.0);
        //Se Intoarce sa vada Vision
        vision.init();
        vision.enable();
        sleep(vision_time);
        vision.disable();
        goldPosition = vision.getTfLite().getLastKnownSampleOrder();
        telemetry.addData("Pozitia este: ", goldPosition);
        sleep(debug_time);
        //Vazut Vision

        //encoderDrive(DRIVE_SPEED,  3,  3, 3.0);
        //robot.mecanumDrive_Cartesian(-0.3d,0);
        //sleep(strafe_time);
        //robot.mecanumDrive_Cartesian(0,0);

        //Merge in fata, ajunge inapoi pe mijloc

        //-------------THE PROGRAM ITSELF-----------------

        //robot.leftClaw.setPosition(1.0);            // S4: Stop and close the claw.
        //robot.rightClaw.setPosition(0.0);
        //sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();

        vision.shutdown();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newLeftTargetBack;
        int newRightTarget;
        int newRightTargetBack;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newLeftTargetBack = robot.leftDriveBack.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newRightTargetBack = robot.rightDriveBack.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.leftDriveBack.setTargetPosition(newLeftTargetBack);
            robot.rightDrive.setTargetPosition(newRightTarget);
            robot.rightDriveBack.setTargetPosition(newRightTargetBack);


            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed));
            robot.leftDriveBack.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed));
            robot.rightDriveBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.leftDriveBack.isBusy() && robot.rightDriveBack.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Fuge spre %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Fuge la %7d :%7d",
                        robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.leftDriveBack.setPower(0);
            robot.rightDrive.setPower(0);
            robot.rightDriveBack.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
