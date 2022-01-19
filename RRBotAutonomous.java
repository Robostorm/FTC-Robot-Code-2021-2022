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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Locale;
import java.util.Comparator;
import java.util.Collections;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

// find team marker and set preloaded cube to the level the marker says
@Autonomous(name="RRBotAutonomous", group="Linear Opmode")
public class RRBotAutonomous extends LinearOpMode {

    // Declare OpMode members.
    RRBotHardware robot = new RRBotHardware();
    RRBotMecanumDrive drive = new RRBotMecanumDrive(robot);
    private ElapsedTime runtime = new ElapsedTime();

    private final double COUNTS_PER_MOTOR_REV = 560 ;    // Andymark 20:1 gearmotor
    private final double DRIVE_GEAR_REDUCTION = 2.0 ;     // This is < 1.0 if geared UP
    private final double WHEEL_DIAMETER_INCHES = 4.0 ;     // For figuring circumference
    private final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    //gyro variables
    private BNO055IMU imu;
    private Orientation angles;

    //vision variables
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    private static final String VUFORIA_KEY =
            "AY+9b5H/////AAABmdP4VHcJckRZm8RJLoHJr6ZCdCwkvYa4e33vyEGuyyl/foBfTYRNT52OO+pJ+FP60SP1HncEL5fgHmD3lbe5XWqlqUt3a6y5hAXpuEDutdVo/n77+mI58Af9ZaBvv9cD2+eXKvwZrFDAEmgZ/+I4OWglTyO2u+zJSNWzA2dLEzM0sPCECY6wR3ytsbff21SAY1MBmVGVjFiAumcc4bdOapJRqXoKHywtduws9uCC3piJGMCqPZmqBTUOnR7myumXyZZWL4TQKfYkcsEKrrlReY0iOgdbTxvDrriljP/FqcoY9UFGenaT6oZ3+/DdfWE+fiarCSzoUdJy+h2BkamY8K4ehsk/bSKG0+qbC8IrQFUl";


    private String cubePos = "";
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5, 16.0/9.0);
        }
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //EncoderDriveTank(1,12,12, 20);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        //EncoderDriveTank(1, 6, -6, 10);
                        //EncoderDriveTank(1, 6, 6, 10);
                        i++;
                    }

                    // Now sort by address instead of name (default).
                    Collections.sort(updatedRecognitions, new Comparator<Recognition>() {
                        public int compare(Recognition one, Recognition other) {
                            if (one.getTop() > other.getTop()) {
                                return -1;
                            } else if (one.getTop() < other.getTop()) {
                                return 1;
                            } else {
                                return 0;
                            }
                        }
                    });
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() > 0) {
                        telemetry.addData("Lowest object", updatedRecognitions.get(0).getTop());
                    }

                    if (updatedRecognitions.size() > 1) {
                        telemetry.addData("2nd Lowest object", updatedRecognitions.get(1).getTop());
                    }

                    if (updatedRecognitions.size() > 2) {
                        telemetry.addData("3rd Lowest object", updatedRecognitions.get(2).getTop());
                    }

                    if (updatedRecognitions.size() >= 2) {
                        Recognition left;
                        Recognition center;
                        if (updatedRecognitions.get(0).getLeft() < updatedRecognitions.get(1).getLeft()) {
                            left = updatedRecognitions.get(0);
                            center = updatedRecognitions.get(1);
                        } else {
                            left = updatedRecognitions.get(1);
                            center = updatedRecognitions.get(0);
                        }
                        if (left.getLabel().equals("Ball") && center.getLabel().equals("Ball")) {
                            telemetry.addData("Ball", "Right");
                            cubePos = "right";
                        } else if (left.getLabel().equals("Cube") && center.getLabel().equals("Cube")) {
                            telemetry.addData("Cube", "Center");
                            cubePos = "center";
                        } else if (left.getLabel().equals("Cube") && center.getLabel().equals("Ball")) {
                            telemetry.addData("Cube", "Left");
                            cubePos = "left";
                        } else {
                            telemetry.addData("Cube", "Stanley is confused");
                            cubePos = "center";
                        }
                    }
                    telemetry.update();
                }
            }
            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.update();
        }
    }

    public void EncoderDriveTank(double speed, double leftInches, double rightInches, double timeoutS)
    {
        robot.rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int newRearLeftTarget;
        int newRearRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive())
        {
            // Determine new target position, and pass to motor controller
            newRearLeftTarget = robot.rearLeftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRearRightTarget = robot.rearRightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newFrontLeftTarget = robot.frontLeftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = robot.frontRightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.rearLeftDrive.setTargetPosition(newRearLeftTarget);
            robot.rearRightDrive.setTargetPosition(newRearRightTarget);
            robot.frontLeftDrive.setTargetPosition(newFrontLeftTarget);
            robot.frontRightDrive.setTargetPosition(newFrontRightTarget);

            // Turn On RUN_TO_POSITION
            robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.rearLeftDrive.setPower(Math.abs(speed));
            robot.rearRightDrive.setPower(Math.abs(speed));
            robot.frontLeftDrive.setPower(Math.abs(speed));
            robot.frontRightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while(opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.rearLeftDrive.isBusy() && robot.rearRightDrive.isBusy() && robot.frontLeftDrive.isBusy() && robot.frontRightDrive.isBusy()))
            {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newRearLeftTarget, newRearRightTarget, newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.rearLeftDrive.getCurrentPosition(),
                        robot.rearRightDrive.getCurrentPosition(),
                        robot.frontLeftDrive.getCurrentPosition(),
                        robot.frontRightDrive.getCurrentPosition());
                telemetry.update();
            }

            TurnOffMotors();

            // Turn off RUN_TO_POSITION
            robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void TurnByGyro(String direction, int angle, double speed)
    {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float startHeading = angles.firstAngle;

        if(direction.equals("left"))
        {
            robot.rearRightDrive.setPower(speed);
            robot.rearLeftDrive.setPower(-speed);
            robot.frontRightDrive.setPower(speed);
            robot.frontLeftDrive.setPower(-speed);
        }
        else if(direction.equals("right"))
        {
            robot.rearRightDrive.setPower(-speed);
            robot.rearLeftDrive.setPower(speed);
            robot.frontRightDrive.setPower(-speed);
            robot.frontLeftDrive.setPower(speed);
        }

        while(opModeIsActive() && Math.abs(angles.firstAngle - startHeading) < angle)
        {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("heading", formatAngle(AngleUnit.DEGREES, angles.firstAngle));
            telemetry.update();
        }

        TurnOffMotors();
    }

    public void TurnOffMotors()
    {
        robot.rearRightDrive.setPower(0);
        robot.rearLeftDrive.setPower(0);
        robot.frontRightDrive.setPower(0);
        robot.frontLeftDrive.setPower(0);
    }

    String formatAngle(AngleUnit angleUnit, double angle)
    {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}
