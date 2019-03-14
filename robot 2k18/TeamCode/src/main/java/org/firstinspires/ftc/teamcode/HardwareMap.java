
/**********     HARDWARE MAP NOU COMPLET AS OF 14.3.2019 ***********/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HardwareMap
{
    /***********  Declarare hardware ***************/

    /* Public OpMode members. */
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor  leftDriveBack = null;
    public DcMotor  rightDriveBack = null;

    public DcMotor foldingMotor0 = null;
    public DcMotor foldingMotor1 = null;

    public DcMotor armMotor = null;
    public DcMotor piedicaMotor= null;

    public CRServo flapperServo = null;

    /***** VITEZE *****/

    //viteza lift
    double liftUpSpeed = 1;
    double liftDownSpeed = -1;
    public double FLAPPER_SPEED = 0.5;


    //viteza miscare pe x/y ,mecanum drive, viteza rotatie
    public double xSpeed = 0.6;
    public double ySpeed = 0.5;
    public double rotateSpeed = 0.3;

    public final double ARM_MOTOR_SPEED=0.5d;


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
        foldingMotor0 = hwMap.get(DcMotor.class, "folding_motor_0");
        foldingMotor1 = hwMap.get(DcMotor.class, "folding_motor_1");
        armMotor = hwMap.get(DcMotor.class, "arm_motor");
        piedicaMotor = hwMap.get(DcMotor.class, "piedica_motor");
        flapperServo = hwMap.get(CRServo.class,"flapper_servo");

        leftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        leftDriveBack.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        rightDriveBack.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        foldingMotor0.setDirection(DcMotor.Direction.REVERSE);// set to REVERSE if using AndyMark motors

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDriveBack.setPower(0);
        rightDriveBack.setPower(0);
        foldingMotor0.setPower(0);
        foldingMotor1.setPower(0);
        armMotor.setPower(0);
        piedicaMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDriveBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        foldingMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        foldingMotor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        piedicaMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        foldingMotor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        foldingMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        piedicaMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    /*****Functii Autonom*****/




 }
