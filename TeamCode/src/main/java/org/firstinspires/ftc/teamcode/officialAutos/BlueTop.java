package org.firstinspires.ftc.teamcode.officialAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Servoconfig;
import org.firstinspires.ftc.teamcode.cameraDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.officialAutos.RobotHardware;
@Autonomous
public class BlueTop extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initHW();
        waitForStart();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    }
}
