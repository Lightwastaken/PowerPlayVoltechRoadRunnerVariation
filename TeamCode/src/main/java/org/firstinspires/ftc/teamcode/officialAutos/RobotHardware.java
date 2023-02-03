/* Copyright (c) 2022 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.officialAutos;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Servoconfig;


/**
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or.. In OnBot Java, add a new file named RobotHardware.java, drawing from this Sample; select Not an OpMode.
 * Also add a new OpMode, drawing from the Sample ConceptExternalHardwareClass.java; select TeleOp.
 *
 */

public class RobotHardware implements Runnable {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotorEx LF   = null; //left front(chassis)
    public DcMotorEx LB   = null; //left back(chassis)
    public DcMotorEx RF  = null; //right front(chassis)
    public DcMotorEx RB  = null; //right back(chassis)
    public DcMotorEx RTL = null; //right motor(lift)
    public DcMotorEx LTL = null; //left motor(lift)
    public Servo claw = null; //claw
    public DistanceSensor sensor = null;

    SensorIMU imuu = new SensorIMU();
    public ElapsedTime runtime = new ElapsedTime();
    public Orientation lastAngles = new Orientation();
    public double currAngle = 0.0;
    public static final int GROUND_OUTTAKE_POSITION = 100;
    public static final int BOTTOM_OUTTAKE_POSITION = 0;
    public static final int LOW_OUTTAKE_POSITION = 550;
    public static final int MID_OUTTAKE_POSITION = 900;
    public static final int TOP_OUTTAKE_POSITION = 1330;
    public static final double OUTTAKE_SPEED = 30 * 5281.1 / 60; //RPM * ENCODER TICKS PER REV / 60
    static final double COUNTS_PER_MOTOR_REV = 5281.1;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 96/25.4;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.7;
    public int targetPosition = 0;

    //PID stuffs
    double integralSum = 0;
    double lastError = 0;
    double derivative;
    double error = getLiftError();
    ElapsedTime timer = new ElapsedTime();
    Thread liftPID = new Thread(new RobotHardware());


    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }
    public RobotHardware() {}


    public void initHW() {
        //INITIALIZE ALL HARDWARE
        LF  = myOpMode.hardwareMap.get(DcMotorEx.class, "LF");
        LB = myOpMode.hardwareMap.get(DcMotorEx.class, "LB");
        RF  = myOpMode.hardwareMap.get(DcMotorEx.class, "RF");
        RB = myOpMode.hardwareMap.get(DcMotorEx.class, "RB");
        RTL = myOpMode.hardwareMap.get(DcMotorEx.class, "RTL");
        LTL = myOpMode.hardwareMap.get(DcMotorEx.class, "LTL");
        claw = myOpMode.hardwareMap.get(Servo.class, "CLAW");
//        sensor = myOpMode.hardwareMap.get(DistanceSensor.class, "distance sensor");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        LF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.REVERSE);
        RTL.setDirection(DcMotorEx.Direction.FORWARD);
        LTL.setDirection(DcMotorEx.Direction.REVERSE);

        //ALL MOTORS RUN WITH ENCODERS
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RTL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LTL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RTL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LTL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        claw.setPosition(1);
        setMotorPowers(0);

        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RTL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LTL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Send telemetry message to signify robot waiting;
    }


/*
    public void setMotorPowers(double leftWheel, double rightWheel) {
        // Output the values to the motor drives.
        LF.setPower(leftWheel);
        LB.setPower(leftWheel);
        RF.setPower(rightWheel);
        RB.setPower(rightWheel);
    }
*/
    public void setMotorPowers(double LFP, double LBP, double RFP, double RBP) {
        // Output the values to the motor drives.
        LF.setPower(LFP);
        LB.setPower(LBP);
        RF.setPower(RFP);
        RB.setPower(RBP);
    }

    public void setMotorPowers(double speed) {
        // Output the values to the motor drives.
        LF.setPower(speed);
        LB.setPower(speed);
        RF.setPower(speed);
        RB.setPower(speed);
    }

    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : myOpMode.hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    public void resetAngle() {
        lastAngles = imuu.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
    }

    public double getAngle() {
        // Get current orientation
        Orientation orientation = imuu.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // Change in angle = current angle - previous angle
        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;
        // Gyro only ranges from -179 to 180
        // If it turns -1 degree over from -179 to 180, subtract 360 from the 359 to get -1
        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }
        // Add change in angle to current angle to get current angle
        currAngle += deltaAngle;
        lastAngles = orientation;
        telemetry.addData("gyro", orientation.firstAngle);
        return currAngle;
    }

    public void turn(double degrees) {
        resetAngle();

        double error = degrees;

        while (linearOpMode.opModeIsActive() && Math.abs(error) > 2) {
            double motorPower = (error < 0 ? -0.2 : 0.2);
            setMotorPowers(-motorPower, motorPower, -motorPower, motorPower);
            error = degrees - getAngle();
            telemetry.addData("error", error);
            telemetry.addData("currentAngle", getAngle());
            System.out.println(error);
            telemetry.update();
        }

        setMotorPowers(0);
    }

    /*
    public void turnTo(double degrees) {

        Orientation orientation = imuu.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double error = degrees - orientation.firstAngle;

        if (error > 180) {
            error -= 360;
        } else if (error < -180) {
            error += 360;
        }

        turn(error);
    }
*/
    public double getAbsoluteAngle() {
        return imuu.imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES
        ).firstAngle;
    }

    /*
    public void encoderDrive(double speed, double leftFront, double rightFront, double leftBack, double rightBack) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;
        int newOuttakeTarget;
        // Ensure that the opmode is still active
        if (linearOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = LF.getCurrentPosition() + (int) (leftFront * COUNTS_PER_INCH);
            newLeftBackTarget = LB.getCurrentPosition() + (int) (leftBack * COUNTS_PER_INCH);
            newRightFrontTarget = RF.getCurrentPosition() + (int) (rightFront * COUNTS_PER_INCH);
            newRightBackTarget = RB.getCurrentPosition() + (int) (rightBack * COUNTS_PER_INCH);
            telemetry.addData("old target", LB.getCurrentPosition());
            LF.setTargetPosition(newLeftFrontTarget);
            LB.setTargetPosition(newLeftBackTarget);
            RF.setTargetPosition(newRightFrontTarget);
            RB.setTargetPosition(newRightBackTarget);
            telemetry.addData("new target", LB.getTargetPosition());
            telemetry.update();


            // Turn On RUN_TO_POSITION
            RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            LF.setPower(Math.abs(speed));
            LB.setPower(Math.abs(speed));
            RF.setPower(Math.abs(speed));
            RB.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (linearOpMode.opModeIsActive() &&
                    (LF.isBusy() || RF.isBusy() || LB.isBusy() || RB.isBusy()))
                telemetry.addData("left front busy", LF.isBusy());
            telemetry.addData("left back busy", LB.isBusy());
            telemetry.addData("right front busy", RF.isBusy());
            telemetry.addData("right back busy", RB.isBusy());
            telemetry.addData("left front pos", LF.getCurrentPosition());
            telemetry.addData("left back pos", LB.getCurrentPosition());
            telemetry.addData("right front pos", RF.getCurrentPosition());
            telemetry.addData("right back pos", RB.getCurrentPosition());
            telemetry.update();
        }

        LF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);
        RF.setPower(0);

        // Turn off RUN_TO_POSITION
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    */
    public void liftEncoderDrive(double speed, double lift) {
        int heightTarget;
        int heightTarget2;
        if (linearOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            heightTarget = LTL.getCurrentPosition() + (int) (lift * COUNTS_PER_INCH);
            heightTarget2 = RTL.getCurrentPosition() + (int) (lift * COUNTS_PER_INCH);
            telemetry.addData("old target", LB.getCurrentPosition());
            LTL.setTargetPosition(heightTarget);
            RTL.setTargetPosition(heightTarget2);


            // Turn On RUN_TO_POSITION
            LTL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RTL.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            LTL.setPower(Math.abs(speed));
            RTL.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

        }

        RTL.setPower(0);
        LTL.setPower(0);

        // Turn off RUN_TO_POSITION
        RTL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LTL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

/*
    public void encoderDrive(double driveSpeed, int i, int i1) {
    }
    public void encoderDrive(double speed, double allMotors) {
        encoderDrive(speed, allMotors, allMotors, allMotors, allMotors);
    }
*/
    public boolean isChassisVeloZero() {
        return LF.getVelocity() == 0 && LB.getVelocity() == 0 && RF.getVelocity() == 0 && RB.getVelocity() == 0;
    }
    public void lift(double power){
        RTL.setPower(power);
        LTL.setPower(power);
    }

    public void moveUp() {
        if (RTL.getCurrentPosition() < LOW_OUTTAKE_POSITION && LTL.getCurrentPosition() < LOW_OUTTAKE_POSITION)
            targetPosition = LOW_OUTTAKE_POSITION;
        else if(RTL.getCurrentPosition() < MID_OUTTAKE_POSITION && LTL.getCurrentPosition() < MID_OUTTAKE_POSITION)
            targetPosition = MID_OUTTAKE_POSITION;
        else if (RTL.getCurrentPosition() < TOP_OUTTAKE_POSITION && LTL.getCurrentPosition() < TOP_OUTTAKE_POSITION)
            targetPosition = TOP_OUTTAKE_POSITION;

        RTL.setTargetPosition(targetPosition);
        LTL.setTargetPosition(targetPosition);
        RTL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LTL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RTL.setVelocity(0.2 * OUTTAKE_SPEED);
        LTL.setVelocity(0.2 * OUTTAKE_SPEED);


        while (LTL.isBusy() && RTL.isBusy() && linearOpMode.opModeIsActive()) {
            linearOpMode.idle();
        }

        while (Math.abs(getLiftError()) > 25) {
            liftPID.start();
            if (getBatteryVoltage() < 10.0) {
                lift(0);
                break;
            }
        }
        telemetry.addData("Lift error", getLiftError());
        telemetry.update();
    }

    public void moveDown() {
        targetPosition = 100;
        RTL.setTargetPosition(targetPosition);
        LTL.setTargetPosition(targetPosition);
        RTL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        LTL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RTL.setVelocity(0.1 * OUTTAKE_SPEED);
        LTL.setVelocity(0.1 * OUTTAKE_SPEED);

        while (LTL.isBusy() && RTL.isBusy() && linearOpMode.opModeIsActive()) {
            linearOpMode.idle();
        }

        while (Math.abs(getLiftError()) > 25) {
            liftPID.start();
            if (getBatteryVoltage() < 10.0) {
                lift(0);
                break;
            }
        }
        telemetry.addData("Lift error", getLiftError());
        telemetry.update();
    }

    public double getLiftError() {
        return RTL.getCurrentPosition() - targetPosition;
    }

    public void run() {
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0.1, 0.1, 0.1, 0);
        double kp = pidfCoefficients.p;
        double ki = pidfCoefficients.i;
        double kd = pidfCoefficients.d;

        derivative = (error - lastError)/timer.seconds();
        integralSum += (error * timer.seconds());
        lift((kp * error) + (ki * integralSum) + (kd * derivative));

        lastError = error;
        timer.reset();
    }
}


