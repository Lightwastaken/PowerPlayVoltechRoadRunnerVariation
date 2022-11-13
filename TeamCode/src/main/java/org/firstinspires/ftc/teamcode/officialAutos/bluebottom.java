package org.firstinspires.ftc.teamcode.officialAutos;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cameraDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

//

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name="bluebottom", group="Pushbot")
public class bluebottom extends LinearOpMode {
    private OpenCvCamera webcam;
    /* Declare OpMode members. */
 RobotHardware robot = new RobotHardware(this);


    // Move in a square
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        cameraDetection detector = new cameraDetection(telemetry);
        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("error");
                telemetry.update();
            }

        });

        robot.initHW();

        robot.LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.setMsTransmissionInterval(50);
        telemetry.addData("status", "ready");
        telemetry.update();

        waitForStart();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();


        while (opModeIsActive()) {
            telemetry.addData("Absolute Angle", robot.getAbsoluteAngle());
            telemetry.addData("Relative Angle", robot.getAngle());
            telemetry.update();
        }


        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

    }



    //*  Method to perform a relative move, based on encoder counts.
    //*  Encoders are not reset as the move is based on the current position.
    //*  Move will stop if any of three conditions occur:
    //*  1) Move gets to the desired position
    //*  2) Move runs out of time
    // *  3) Driver stops the opmode running.

    // Wait for `msTime` milliseconds

    // *** Turning ***

    // resets currAngle Value



}
