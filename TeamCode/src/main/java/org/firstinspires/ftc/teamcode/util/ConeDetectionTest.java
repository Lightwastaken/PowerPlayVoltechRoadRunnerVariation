package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.VISION.Vision;
import org.firstinspires.ftc.teamcode.officialAutos.RobotHardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.util.ConeDetection;
import org.openftc.easyopencv.OpenCvPipeline;
import java.lang.Math;

@Autonomous(name = "cone alignment test", group = "Pushbot")
public class ConeDetectionTest extends LinearOpMode {
    ConeDetection coneDetection;
    ElapsedTime timer = new ElapsedTime();

    public void runOpMode() {
        RobotHardware robot = new RobotHardware(this);
        robot.initHW();

        ConeDetection cam = new ConeDetection(telemetry);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera.setPipeline(coneDetection);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            public void onOpened() { camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT); }

            public void onError(int errorCode) {}
        });

        while(Math.abs(cam.getOffset()) > 50) {
            robot.chassisSetPower(-0.1 * Math.abs(cam.getOffset())/cam.getOffset());
        }

    }
}
