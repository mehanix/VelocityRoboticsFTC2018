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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Pushbot: diy mecanum", group="Pushbot")

public class mecanum_diy extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareMap robot           = new HardwareMap();   // Use a Pushbot's hardware


    double xSpeed = 0.4;
    double ySpeed = 0.4;
    double rotateSpeed = 0.2;
    @Override
    public void runOpMode() {

        double xValue,yValue;

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            xValue = gamepad1.right_stick_x;
            yValue = gamepad1.left_stick_y;

            //normalizare
            if (xValue > 1.0) xValue=1.0;
            if (yValue > 1.0) yValue=1.0;
            if (xValue < -1.0) xValue=-1.0;
            if (xValue < -1.0) xValue=-1.0;

            double rotationLeft = gamepad1.left_trigger;
            double rotationRight = gamepad1.right_trigger;
            if(rotationLeft==0 && rotationRight == 0)
                mecanumDrive_Cartesian(xValue,yValue);
            else
                mecanumDrive_Rotate(rotationLeft,rotationRight);

            // Send telemetry message to signify robot running;

            telemetry.addData("x",  "%.2f", xValue);
            telemetry.addData("y", "%.2f", yValue);
            telemetry.update();

        }
    }

    public void mecanumDrive_Cartesian(double x, double y)
    {
        if(x!=0) {
            double aux = x > 0 ? -xSpeed : xSpeed;
            robot.leftDrive.setPower(aux);
            robot.rightDrive.setPower(-aux);
            robot.leftDriveBack.setPower(-aux);
            robot.rightDriveBack.setPower(aux);
        }

        else if(y!=0) {
            double aux = y > 0 ? ySpeed : -ySpeed;
            robot.leftDrive.setPower(aux);
            robot.rightDrive.setPower(aux);
            robot.leftDriveBack.setPower(aux);
            robot.rightDriveBack.setPower(aux);
        }

        if(x==0 && y==0) {
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
            robot.leftDriveBack.setPower(0);
            robot.rightDriveBack.setPower(0);

        }


        /*
            double wheelSpeeds[] = new double[4];

            wheelSpeeds[0] = x + y + rotation;
            wheelSpeeds[1] = -x + y - rotation;
            wheelSpeeds[2] = -x + y + rotation;
            wheelSpeeds[3] = x + y - rotation;

            normalize(wheelSpeeds);

            robot.leftDrive.setPower(wheelSpeeds[0]);
            robot.rightDrive.setPower(wheelSpeeds[1]);
            robot.leftDriveBack.setPower(wheelSpeeds[2]);
            robot.rightDriveBack.setPower(wheelSpeeds[3]);
        */
    }   //mecanumDrive_Cartesian


    public void mecanumDrive_Rotate(double rotateLeft, double rotateRight)
    {
        //intai stanga dupa dreapta
        double direction = rotateLeft > rotateRight ? 1 : -1 ;
        robot.leftDrive.setPower(rotateSpeed*-1*direction);
        robot.rightDrive.setPower(rotateSpeed*direction);
        robot.leftDriveBack.setPower(rotateSpeed*-1*direction);
        robot.rightDriveBack.setPower(rotateSpeed*direction);

    }
    private void normalize(double[] wheelSpeeds)
    {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);

        for (int i = 1; i < wheelSpeeds.length; i++)
        {
            double magnitude = Math.abs(wheelSpeeds[i]);

            if (magnitude > maxMagnitude)
            {
                maxMagnitude = magnitude;
            }
        }

        if (maxMagnitude > 1.0)
        {
            for (int i = 0; i < wheelSpeeds.length; i++)
            {
                wheelSpeeds[i] /= maxMagnitude;
            }
        }
    }   //normalize
}
