package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.HardwareMap;

@TeleOp(name="VisionJavaExample", group="Autonom")
public class VisionJavaExample extends LinearOpMode{
    HardwareMap robot = new HardwareMap();

    MasterVision vision;
    SampleRandomizedPositions goldPosition;

    int MINERAL_DRIVE_TIME=1500;
    int ROTATE_TIME=1300;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap); /****** IMPORTANT LOL *****/

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;// recommended camera direction
        parameters.vuforiaLicenseKey = "AdCqxpv/////AAAAmRYY0W12w0kCiohldJxGqHFZ2GletPTfhXtZhESwlq9pxkGaOfOwJah+6BGckYD4jJ5cmiwelBvWWK7mdEDxpkxkAFJmyxqYcUymQ8BMWBSIsoxbJIWwJ6XeYIRecso9jVV3iF1hgUPJ47uYiB/N7GuqM3DnD+uogQpHCMsy+KS6l0yJSGSTdlGpUu3lwxmVKpjc0Ox0biDe5VdmWNrJJ6DoIuy4TCzectZfidTbMJXwfigDK7dxNLicETcE0RrHgpSQ3F2YWn/ZOoyNZk2JmkPzu4EKPSHIWQfOiIU3KPHP9cjqh7x3NHuvvdyIYJjNb6xpzoA29YQK7l4zNRma0dQSBTyuiSdfkjH/6QSuqiBR";
        //din vazut doar 2, dat seama ce e al 3lea :)
        vision = new MasterVision(parameters, hardwareMap, true, MasterVision.TFLiteAlgorithm.INFER_LEFT);
        vision.init();// enables the camera overlay. this will take a couple of seconds
        vision.enable();// enables the tracking algorithms. this might also take a little time

        waitForStart();

        vision.disable();// disables tracking algorithms. this will free up your phone's processing power for other jobs.

        goldPosition = vision.getTfLite().getLastKnownSampleOrder();

            telemetry.addData("goldPosition was", goldPosition);// giving feedback



            switch (goldPosition){ // using for things in the autonomous program
                case LEFT:
                {
                    telemetry.addLine("going to the left");
                    robot.mecanumDrive_Rotate(1d,0);
                    sleep(ROTATE_TIME);
                    robot.mecanumDrive_Rotate(0,0);
                    sleep(250);

                    robot.mecanumDrive_Cartesian(-1d,0);
                    sleep(MINERAL_DRIVE_TIME);
                    robot.mecanumDrive_Cartesian(0,0);
                    break;
                }
                case CENTER: {
                    telemetry.addLine("going straight");
                    robot.mecanumDrive_Cartesian(-1d,0);
                    sleep(MINERAL_DRIVE_TIME-250);
                    robot.mecanumDrive_Cartesian(0,0);
                    break;
                }
                case RIGHT:
                {
                    telemetry.addLine("going to the right");
                    robot.mecanumDrive_Rotate(0,1d);
                    sleep(ROTATE_TIME);
                    robot.mecanumDrive_Rotate(0,0);
                    sleep(250);
                    robot.mecanumDrive_Cartesian(-1d,0);
                    sleep(MINERAL_DRIVE_TIME);
                    robot.mecanumDrive_Cartesian(0,0);
                }
                    break;
                case UNKNOWN:
                    telemetry.addLine("staying put");
                    break;
            }

            telemetry.update();


        vision.shutdown();
    }


    public void mecanumDrive_Cartesian(double x, double y)
    {
        //verif input fata/spate
        if(x!=0) {

            double aux = x > 0 ? robot.xSpeed : -robot.xSpeed;
            robot.leftDrive.setPower(aux);
            robot.rightDrive.setPower(-aux);
            robot.leftDriveBack.setPower(-aux);
            robot.rightDriveBack.setPower(aux);
        }

        //verif input stanga/dreapta
        else if(y!=0) {

            double aux = y > 0 ? -robot.ySpeed : robot.ySpeed;
            robot.leftDrive.setPower(aux);
            robot.rightDrive.setPower(aux);
            robot.leftDriveBack.setPower(aux);
            robot.rightDriveBack.setPower(aux);
        }

        //daca niciuna, stop!!!!!
        if(x==0 && y==0) {
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
            robot.leftDriveBack.setPower(0);
            robot.rightDriveBack.setPower(0);

        }
    }   //mecanumDrive_Cartesian


    public void mecanumDrive_Rotate(double rotateLeft, double rotateRight)
    {
        //intai stanga dupa dreapta
        double direction = rotateLeft > rotateRight ? 1 : -1 ;
        robot.leftDrive.setPower(robot.rotateSpeed*-1*direction);
        robot.rightDrive.setPower(robot.rotateSpeed*direction);
        robot.leftDriveBack.setPower(robot.rotateSpeed*-1*direction);
        robot.rightDriveBack.setPower(robot.rotateSpeed*direction);

    }

}
