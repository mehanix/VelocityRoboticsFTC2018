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
import com.qualcomm.robotcore.util.Range;

/*********** TELEOP: MECANUM DRIVE ***********/

@TeleOp(name="Pushbot: diy mecanum", group="Teleop")

public class mecanum_diy extends LinearOpMode {

    /* Declare OpMode members. */

    /*********** INIT VALORI **********/


    HardwareMap robot  = new HardwareMap();   //HardwareMap al robotului nostru



    double extMotorFrontSpeedSlow = 0.3;
    double getExtMotorFrontSpeedFast = 0.6;
    double extMotorBackSpeed = 0.4;
    float forward = 1;
    float backward = -1;


    double ArmPosition = robot.Arm_down;
    final double Arm_speed = 1;

    @Override
    public void runOpMode() {

        double xValue,yValue;

        robot.init(hardwareMap);




        telemetry.addData("Say", "Hello Driver! Totul este initializat");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /******* Citit valori de pe gamepad ********/

            xValue = gamepad1.right_stick_x;
            yValue = gamepad1.left_stick_y;
            boolean extensionFrontForward = gamepad1.dpad_left;
            boolean extensionFrontBackward = gamepad1.dpad_right;
            boolean extensionBackForward = gamepad1.dpad_up;
            boolean extensionBackBackward = gamepad1.dpad_down;

            boolean liftUp = gamepad2.dpad_up;
            boolean liftDown = gamepad2.dpad_down;

            double rotationLeft = gamepad1.left_trigger;
            double rotationRight = gamepad1.right_trigger;


            /********* TELEMETRY: mesaje debug (apar pe ecranul telefonului de driver) *********/

            //THE TEXT (FOR DEBUGGING PURPOSES)
            /** DRIVER 1: miscare, rotatie, extins mana rampa automat/manual, control flappere **/

            telemetry.addLine("Driver 1 (Ground Movement):");

            //Basic Controls (Forward, Backward, LeftMechano, RightMechano)
            telemetry.addData("Y axis", "%.2f", yValue);
            telemetry.addData("X axis",  "%.2f", xValue);


            //Turning (Tank-style)
            telemetry.addData("left_trigger button:",gamepad1.left_trigger);
            telemetry.addData("right_trigger button:",gamepad1.right_trigger);

            //Extension Arm (of the ramp)
            telemetry.addData("dpad up:",extensionBackForward);
            telemetry.addData("dpad down:",extensionBackBackward);


            //The Ramp (Manual Controls)
            telemetry.addData("dpad left:",extensionFrontForward);
            telemetry.addData("dpad right:",extensionFrontBackward);

            //The Ramp (Automatic, down/up switch)
            telemetry.addData("a button:",gamepad1.a);


            //The Flappers (Reversing the rotation)
            telemetry.addData("b button:",gamepad1.b);


            //The Flappers (On/Off)
            telemetry.addData("x button:",gamepad1.x);
            telemetry.addLine(" ");

            /**DRIVER 2: control lift, aruncare cuburi**/
            telemetry.addLine("Driver 2 (Lift Movement):");

            //The Lift
            telemetry.addData("dpad up",gamepad1.dpad_up);
            telemetry.addData("dpad down",gamepad1.dpad_down);
            //The Throwing Arm
            telemetry.addData("a",gamepad1.a);
            telemetry.addData("b",gamepad1.b);

            telemetry.update();

            //normalizare
            if (xValue > 1.0) xValue=1.0;
            if (yValue > 1.0) yValue=1.0;
            if (xValue < -1.0) xValue=-1.0;
            if (xValue < -1.0) xValue=-1.0;



            /****** CONTROL MISCARE : 4 directii (fata, spate, stanga dreapta), rotatie - mecanum drive *******/
            if(rotationLeft==0 && rotationRight == 0)
                robot.mecanumDrive_Cartesian(xValue,yValue);
            else
                robot.mecanumDrive_Rotate(rotationLeft,rotationRight);




            if( extensionBackForward == false && extensionBackBackward == false) {
                robot.extensionMotorBack.setPower(0);
            }
            else {
                if(extensionBackForward == true)
                    powerExtensionMotorBack(forward);
                else powerExtensionMotorBack(backward);
            }

            if( extensionFrontForward == false && extensionFrontBackward == false){
                robot.extensionMotorFront.setPower(0);
            }
            else {
                if(extensionFrontForward == true)
                    powerExtensionMotorFront(forward,extMotorFrontSpeedSlow);
                else powerExtensionMotorFront(backward,getExtMotorFrontSpeedFast);
            }

            if(gamepad1.a == true)
                robot.changeFrontMotorState(this);

            if(gamepad1.b == true)
                robot.changeFlappersRotation(this);
            if(gamepad1.x == true)
            {
                robot.powerFlappers(this);
            }

            if(liftUp == true)
                robot.lift.setPower(robot.liftUpSpeed);
                else
                robot.lift.setPower(0);
            if(liftDown == true)
                robot.lift.setPower(robot.liftDownSpeed);
                else
                robot.lift.setPower(0);

            if(gamepad2.a == true)
            {
                ArmPosition += Arm_speed;
            }
            else if(gamepad2.b == true)
            {
                ArmPosition -= Arm_speed;
            }
            ArmPosition = Range.clip(ArmPosition,robot.Arm_down,robot.Arm_up);
            robot.ArmL.setPosition(ArmPosition);
            robot.ArmR.setPosition(ArmPosition);
            // Send telemetry message to signify robot running;


        }
    }

    public void powerExtensionMotorBack(float direction)
    {
        robot.extensionMotorBack.setPower(extMotorBackSpeed*direction);
    }

    public void powerExtensionMotorFront(float direction, double speed)
    {
        robot.extensionMotorFront.setPower(speed*direction);
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
