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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareMap
{
    /* Public OpMode members. */
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor  leftDriveBack = null;
    public DcMotor  rightDriveBack = null;

    public DcMotor  extensionMotorFront =null;
    public DcMotor  extensionMotorBack =null;
    public String frontMotorCurrentState = "up";
    public boolean changeInProgress=false;
    public float extensionMotorFront_forwardSpeed= 0.3f;
    public float extensionMotorFront_backSpeed= -0.5f;

    public DcMotor constantFlappers = null;
    public String flappersState = "take";
    double constantFlappers_takeSpeed= 0.5f;
    double constantFlappers_giveSpeed= -0.5f;
    String flappersPower = "off";

    public DcMotor lift = null;

    public Servo armL = null;
    public Servo armR = null;
    public final static double arm_down = 0.0;
    public final static double arm_up = 1.0;

    /** 180grade=1.0 => 0.1=18grade; 0.05=9grade; **/
    public Servo smallArm = null;
    public final static double smallArm_unextended = 0.37;
    public final static double smallArm_extended = 0.8;

    public Servo liftBrake = null;
    public final static double liftBrake_locked = 0.2;
    public final static double liftBrake_unlocked = 0.5;

    /***** VITEZE *****/

    //viteza lift
    double liftUpSpeed = 2;
    double liftDownSpeed = -2;


    //viteza miscare pe x/y ,mecanum drive, viteza rotatie
    public double xSpeed = 0.6;
    public double ySpeed = 0.5;
    public double rotateSpeed = 0.3;


    /* local OpMode members. */
    com.qualcomm.robotcore.hardware.HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareMap(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(com.qualcomm.robotcore.hardware.HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        leftDriveBack  = hwMap.get(DcMotor.class, "left_drive_back");
        rightDriveBack = hwMap.get(DcMotor.class, "right_drive_back");
        extensionMotorFront = hwMap.get(DcMotor.class, "extension_motor_front");
        extensionMotorBack = hwMap.get(DcMotor.class, "extension_motor_back");
        constantFlappers = hwMap.get(DcMotor.class, "constant_flappers");
        lift = hwMap.get(DcMotor.class, "lift");


        leftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        leftDriveBack.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        rightDriveBack.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        extensionMotorFront.setDirection(DcMotor.Direction.REVERSE);
        extensionMotorBack.setDirection(DcMotor.Direction.REVERSE);
        extensionMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        constantFlappers.setDirection(DcMotor.Direction.FORWARD);
        constantFlappers.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotor.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDriveBack.setPower(0);
        rightDriveBack.setPower(0);
        extensionMotorFront.setPower(0);
        extensionMotorBack.setPower(0);
        constantFlappers.setPower(0);
        lift.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDriveBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensionMotorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensionMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        constantFlappers.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Define and initialize ALL installed servos.
        armL = hwMap.get(Servo.class, "armL");
        armR = hwMap.get(Servo.class, "armR");
        armL.setDirection(Servo.Direction.REVERSE);
        armR.setDirection(Servo.Direction.FORWARD);
        armL.setPosition(arm_down);
        armR.setPosition(arm_down);

        smallArm = hwMap.get(Servo.class, "smallArm");
        liftBrake = hwMap.get(Servo.class, "liftBrake");
        smallArm.setDirection(Servo.Direction.REVERSE);  //ToTest -----------------------------
        liftBrake.setDirection(Servo.Direction.FORWARD); //ToTest -----------------------------
        smallArm.setPosition(smallArm_unextended);
        liftBrake.setPosition(liftBrake_locked);

    }

    public void mecanumDrive_Cartesian(double x, double y)
    {
        //verif input fata/spate
        if(x!=0) {

            double aux = x > 0 ? xSpeed : -xSpeed;
            leftDrive.setPower(aux);
            rightDrive.setPower(-aux);
            leftDriveBack.setPower(-aux);
            rightDriveBack.setPower(aux);
        }

        //verif input stanga/dreapta
        else if(y!=0) {

            double aux = y > 0 ? -ySpeed : ySpeed;
            leftDrive.setPower(aux);
            rightDrive.setPower(aux);
            leftDriveBack.setPower(aux);
            rightDriveBack.setPower(aux);
        }

        //daca niciuna, stop!!!!!
        if(x==0 && y==0) {
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            leftDriveBack.setPower(0);
            rightDriveBack.setPower(0);

        }
    }   //mecanumDrive_Cartesian


    public void mecanumDrive_Rotate(double rotateLeft, double rotateRight)
    {
        //intai stanga dupa dreapta
        double direction = rotateLeft > rotateRight ? 1 : -1 ;
        leftDrive.setPower(rotateSpeed*-1*direction);
        rightDrive.setPower(rotateSpeed*direction);
        leftDriveBack.setPower(rotateSpeed*-1*direction);
        rightDriveBack.setPower(rotateSpeed*direction);

    }




    public void changeFlappersRotation(LinearOpMode opMode)
    {
        if(flappersState=="take")
            flappersState="give";
            else
            flappersState="take";
        opMode.sleep(200);
        if(flappersPower == "on")
        {
            if(flappersState=="take")
                constantFlappers.setPower(constantFlappers_takeSpeed);
            else
                constantFlappers.setPower(constantFlappers_giveSpeed);
        }
    }

    public void powerFlappers(LinearOpMode opMode)
    {
        if(flappersPower == "off")
        {
            flappersPower = "on";
            if(flappersState=="take")
                constantFlappers.setPower(constantFlappers_takeSpeed);
                else
                constantFlappers.setPower(constantFlappers_giveSpeed);
        }
        else
        {
            flappersPower = "off";
            constantFlappers.setPower(0);
        }
        opMode.sleep(200);
    }

    public void changeFrontMotorState(LinearOpMode opMode)
    {
        //daca nu se schimba deja state-ul motorului
        if(changeInProgress==false)
        {
            //anunta ca e in schimbare
            changeInProgress=true;

            //daca ii urcat, coboara-l
            if(frontMotorCurrentState=="up")
            {
                extensionMotorFront.setPower(extensionMotorFront_forwardSpeed);
                opMode.sleep(700);
                extensionMotorFront.setPower(0);
                frontMotorCurrentState="down";


            }
            else
            {
                //daca e coborat, urca-l
                extensionMotorFront.setPower(extensionMotorFront_backSpeed);
                opMode.sleep(900);
                extensionMotorFront.setPower(0);
                frontMotorCurrentState="up";
            }
            changeInProgress = false;
        }

    }


    /*****Functii Autonom*****/


    //Functie dat jos de pe carlig!
    public void unlatch(LinearOpMode op) {

        liftBrake.setPosition(liftBrake_unlocked);
        op.sleep(500);
        lift.setPower(liftDownSpeed);
        op.sleep(1000);
        lift.setPower(0);

        /***TODO FACUT FUNCTIE mecanumDrive_Autonom !!!!!!!! cu encodere
         *  Ca nu o sa mearga facut la ochi asta***/
        mecanumDrive_Cartesian(1,0);
        op.sleep(1000);
        mecanumDrive_Cartesian(0,0);

    }




 }

