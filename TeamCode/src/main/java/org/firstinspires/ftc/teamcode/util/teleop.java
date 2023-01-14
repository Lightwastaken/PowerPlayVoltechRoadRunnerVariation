package org.firstinspires.ftc.teamcode.util;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.officialAutos.RobotHardware;

@TeleOp(name="teleoptoggles", group="Pushbot")
public class teleop extends LinearOpMode {

    /* Declare OpMode members. */
    ;   // Use a Pushbot's hardware
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {

        //checking teleop distance if encoders convert to inches correctly
        final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder
        final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 96.0 / 25.4;     // For figuring circumference
        final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        boolean active = true;

        double left;
        double right;
        double max;
        double speedControl;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.initHW();
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {


            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            double y = -gamepad1.right_stick_y; // Remember, this is reversed!
            double x = gamepad1.right_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.left_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1 );
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            robot.LF.setPower(frontLeftPower);
            robot.LB.setPower(backLeftPower);
            robot.RF.setPower(frontRightPower);
            robot.RB.setPower(backRightPower);

            // non toggle claws
            if (gamepad1.left_bumper && active == false) {
                active = true;
                robot.claw.setPosition(1);

            }

            if (gamepad1.left_bumper && active == true){
                active = false;
                robot.claw.setPosition(0);
            }


            if (gamepad1.right_trigger > 0.1) {
                robot.lift(0.2);
            } else if (gamepad1.right_bumper) {
                robot.lift(-0.2);
            } else {
                robot.lift(0);

                telemetry.addData("LF Encoder", robot.LF.getCurrentPosition());
                telemetry.addData("LB Encoder", robot.LB.getCurrentPosition());
                telemetry.addData("RF Encoder", robot.RF.getCurrentPosition());
                telemetry.addData("RB Encoder", robot.RB.getCurrentPosition());
                telemetry.addData("LF Inches", robot.LF.getCurrentPosition() / COUNTS_PER_INCH);
                telemetry.addData("LB Inches", robot.LB.getCurrentPosition() / COUNTS_PER_INCH);
                telemetry.addData("RF Inches", robot.RF.getCurrentPosition() / COUNTS_PER_INCH);
                telemetry.addData("RB Inches", robot.RB.getCurrentPosition() / COUNTS_PER_INCH);
                telemetry.update();
//
//                // Pace this loop so jaw action is reasonable speed.
                sleep(50);
            }

        }

    }
    public void clawPosition (double position) {
        robot.claw.setPosition(position);
    }

}

