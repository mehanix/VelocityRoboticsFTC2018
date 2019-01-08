package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@TeleOp
public class VisionJavaExample extends LinearOpMode{
    MasterVision vision;
    SampleRandomizedPositions goldPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;// recommended camera direction
        parameters.vuforiaLicenseKey = "Ab5Nb6v/////AAABmazPLYroFk4kj/5KzHz0OlFPZlSgMgz0S0v1EYRYw6ki4nopZkTtHGFpNPlX2iscXIAmfnyyRlo4Ef7D3tvZAEu5zUkA783DpnU8/kgyjpOO0/C51lh/jBnltN0b/TdlWyfiyIngtGBfNx+WZdmDk5uK3V/Bc95gHWQT6AR5ZyZYhC5BbjiVuVaOwQGANP3qYgydbo3Vg7td0oB8vxNI4uMFTYH6/r1Z+6cfj4e7+GXwHwsj/nvQA7hjHaoMhfg73hfcDQbj0IdZBRG9sEyPpHAOE3FKPdfvNeTtVravUzKT81pS8d2jsLvNyjqJx6tpiGoN8bqQyCEQfeTUJOd7aQBpFJoHoLPBBWb48EGKmbe3\n";

        //din vazut doar 2, dat seama ce e al 3lea :)
        vision = new MasterVision(parameters, hardwareMap, true, MasterVision.TFLiteAlgorithm.INFER_LEFT);
        vision.init();// enables the camera overlay. this will take a couple of seconds
        vision.enable();// enables the tracking algorithms. this might also take a little time

        waitForStart();

        vision.disable();// disables tracking algorithms. this will free up your phone's processing power for other jobs.

        goldPosition = vision.getTfLite().getLastKnownSampleOrder();

        while(opModeIsActive()){
            telemetry.addData("goldPosition was", goldPosition);// giving feedback

            switch (goldPosition){ // using for things in the autonomous program
                case LEFT:
                    telemetry.addLine("going to the left");
                    break;
                case CENTER:
                    telemetry.addLine("going straight");
                    break;
                case RIGHT:
                    telemetry.addLine("going to the right");
                    break;
                case UNKNOWN:
                    telemetry.addLine("staying put");
                    break;
            }

            telemetry.update();
        }

        vision.shutdown();
    }
}
