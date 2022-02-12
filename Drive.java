package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Drive {

    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public static void encoder(double speed, double distance, double timeoutS, LinearOpMode opmode, RRBotHardware robot) {
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        // Ensure that the opmode is still active
        if (opmode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int newRightTarget = robot.frontRightDrive.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            int newLeftTarget = robot.frontLeftDrive.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

            if (distance > 0) {
                robot.frontRightDrive.setPower(speed);
                robot.frontLeftDrive.setPower(speed);
                robot.rearRightDrive.setPower(speed);
                robot.rearLeftDrive.setPower(speed);

                while (opmode.opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (robot.frontRightDrive.getCurrentPosition() < newRightTarget && robot.frontLeftDrive.getCurrentPosition() < newLeftTarget)) {

                    // Display it for the driver.
                    opmode.telemetry.addData("Encoder Target", "%7d :%7d", newRightTarget, newLeftTarget);
                    opmode.telemetry.addData("Current Encoder", "%7d :%7d",
                            robot.frontRightDrive.getCurrentPosition(),
                            robot.frontLeftDrive.getCurrentPosition());
                    opmode.telemetry.update();
                }
            } else {
                robot.frontRightDrive.setPower(-speed);
                robot.frontLeftDrive.setPower(-speed);
                robot.rearRightDrive.setPower(-speed);
                robot.rearLeftDrive.setPower(-speed);


                while (opmode.opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (robot.frontRightDrive.getCurrentPosition() > newRightTarget && robot.frontLeftDrive.getCurrentPosition() > newLeftTarget)) {

                    // Display it for the driver.
                    opmode.telemetry.addData("Encoder Target", "%7d :%7d", newRightTarget, newLeftTarget);
                    opmode.telemetry.addData("Current Encoder", "%7d :%7d",
                            robot.frontRightDrive.getCurrentPosition(),
                            robot.frontLeftDrive.getCurrentPosition());
                    opmode.telemetry.update();
                }
            }

            robot.frontRightDrive.setPower(0.0);
            robot.frontLeftDrive.setPower(0.0);
            robot.rearRightDrive.setPower(0.0);
            robot.rearLeftDrive.setPower(0.0);

            //  sleep(250);   // optional pause after each move
        }
    }

}