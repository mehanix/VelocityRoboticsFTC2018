
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/*********** TELEOP: MECANUM DRIVE ***********/

@TeleOp(name="Teleop Robot", group="Teleop")

public class TeleOpDrive extends LinearOpMode {

    /* Declare OpMode members. */

    /*********** INIT VALORI **********/


    HardwareMap robot = new HardwareMap();   //HardwareMap al robotului nostru



    public String customSpeed = "off";

    @Override
    public void runOpMode() {

        double xValue, yValue;
        robot.init(hardwareMap);

        telemetry.addData("Say", "Hello Driver! Totul este initializat ");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /******* Citit valori de pe gamepad ********/

            xValue = gamepad1.right_stick_x;
            yValue = gamepad1.left_stick_y;

            //boolean liftUp = gamepad2.dpad_up;
            //boolean liftDown = gamepad2.dpad_down;

            double rotationLeft = gamepad1.left_trigger;
            double rotationRight = gamepad1.right_trigger;


            double armFront = gamepad2.right_trigger;
            double armBack = gamepad2.left_trigger;

            boolean flapperOn = gamepad2.a;
            boolean flapperRev = gamepad2.b;
            boolean flapperOff = gamepad2.x;


            /********* TELEMETRY: mesaje debug (apar pe ecranul telefonului de driver) *********/

            //THE TEXT (FOR DEBUGGING PURPOSES)
            /** DRIVER 1: Miscare, Brat Marker, Piedica Lift **/

            telemetry.addLine("LOL");
            telemetry.addLine("Driver 1 (Ground Movement):");

            //Basic Controls (Forward, Backward, LeftMechano, RightMechano)
            telemetry.addData("Y axis", "%.2f", yValue);
            telemetry.addData("X axis", "%.2f", xValue);


            //Turning (Tank-style)
            telemetry.addData("left_trigger button:", gamepad1.left_trigger);
            telemetry.addData("right_trigger button:", gamepad1.right_trigger);

            //Extension Arm (of the ramp)
            telemetry.addData("Extension Arm", gamepad1.left_stick_y);


            //The Ramp (Manual Controls)
            telemetry.addData("Ramp", gamepad1.right_stick_y);
        /**DRIVER 2: control lift, aruncare cuburi, debugging/deUrgenta**/
            telemetry.addLine("Driver 2 (Lift Movement):");

            //The Lift
            telemetry.addData("dpad up", gamepad2.dpad_up);
            telemetry.addData("dpad down", gamepad2.dpad_down);
            //The Throwing Arm
            telemetry.addData("a", gamepad2.a);
            telemetry.addData("b", gamepad2.b);

            telemetry.update();

            //normalizare
            if (xValue > 1.0) xValue = 1.0;
            if (yValue > 1.0) yValue = 1.0;
            if (xValue < -1.0) xValue = -1.0;
            if (xValue < -1.0) xValue = -1.0;


            /****** CONTROL MISCARE : 4 directii (fata, spate, stanga dreapta), rotatie - mecanum drive *******/
            if (rotationLeft == 0 && rotationRight == 0)
                robot.mecanumDrive_Cartesian(xValue, yValue);
            else
                robot.mecanumDrive_Rotate(rotationLeft, rotationRight);


            if (gamepad1.left_bumper == true) {
                robot.xSpeed = 1.0;      //Speeedoo Moodoo
                robot.ySpeed = 0.9;
                robot.rotateSpeed = 0.5;
            } else if (gamepad1.right_bumper == true) {
                robot.xSpeed = 0.3;      //Sneeakoo Moodoo
                robot.ySpeed = 0.2;
                robot.rotateSpeed = 0.1;
            } else {
                robot.xSpeed = 0.6;      //Normaloo Moodoo
                robot.ySpeed = 0.5;
                robot.rotateSpeed = 0.3;
            }


            /*****  arm forward/backward  ****/


            if (armFront > 0)
            {
                robot.armMotor.setPower(robot.ARM_MOTOR_SPEED);
            }
            else
            {
                robot.armMotor.setPower(0);

            }
            if (armBack > 0)
            {
                robot.armMotor.setPower(-robot.ARM_MOTOR_SPEED);
            }
            else
            {
                robot.armMotor.setPower(0);

            }


            if(flapperOn == true)
                robot.flapperServo.setPower(robot.FLAPPER_SPEED);
            if(flapperOff == true)
                robot.flapperServo.setPower(0);
            if(flapperRev == true)
            {
                if(robot.flapperServo.getDirection()== CRServo.Direction.FORWARD)
                    robot.flapperServo.setDirection(CRServo.Direction.REVERSE);
                else
                    robot.flapperServo.setDirection(CRServo.Direction.FORWARD);
            }


            /** Emergency/Debug controls **/


            telemetry.addLine("-Debug Controls-");


            // Send telemetry message to signify robot running;


        }
    }


    private void normalize(double[] wheelSpeeds) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);

        for (int i = 1; i < wheelSpeeds.length; i++) {
            double magnitude = Math.abs(wheelSpeeds[i]);

            if (magnitude > maxMagnitude) {
                maxMagnitude = magnitude;
            }
        }

        if (maxMagnitude > 1.0) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] /= maxMagnitude;
            }
        }
    }   //normalize

}
