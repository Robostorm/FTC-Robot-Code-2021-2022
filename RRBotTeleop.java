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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static android.os.SystemClock.sleep;

@TeleOp(name="Teleop")
public class RRBotTeleop extends OpMode{

    /* Declare OpMode members. */
    RRBotHardware robot       = new RRBotHardware(); // use the class created to define a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    //construct drive class
    RRBotMecanumDrive drive = new RRBotMecanumDrive(robot);

    // Arm Variables
    double lastArmMove = 0;
    double lastIntake = 0;

    boolean carouselRotatorOn = false;
    boolean intakeOn = false;
    int armPosition = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Indicate to the driver the robot is initialized and ready to start
        telemetry.addData("Status", "Initialized");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        DriveUpdate();

        ArmUpdate();

        intakeUpdate();

        CarouselRotatorUpdate();

        Telemetry();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    /**
     * Updates the drive system with manual and automatic movements
     */
    public void DriveUpdate(){
        if(!drive.getIsAutoMove()){
            drive.setMotorPower(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, -gamepad1.right_stick_y, true);
        } else{
            drive.AutoMoveEndCheck();
        }
    }

    /**
     * Arm Positions:
     * 0 - Lowest position, used when picking up freight
     * 1 - Used when placing freight in shared shipping hub or lowest level or blue or red shipping hubs
     * 2 - Used when placing freight in the 2nd level of the blue or red shipping hubs
     * 3 - Used when placing freight in the 3rd and highest level of the blue or red shipping hubs
     * 4 - Arm folds to fit within 18x18x18 sizing constrains
     */
    // Sets moves the arm between 4 preset positions
    public void ArmUpdate(){
        // Increases the armPosition by one every time dpad up is pressed
        if (gamepad1.dpad_up && armPosition < 4 && runtime.time() - lastArmMove > 0.5) {
            armPosition += 1;
            lastArmMove = (double) runtime.time();
        }
        // Decreases the armPosition variable by one every time dpad down is pressed
        else if (gamepad1.dpad_down && armPosition > 0 && runtime.time() - lastArmMove > 0.5) {
            armPosition -= 1;
            lastArmMove = (double) runtime.time();
        }

        // Sets the position of the arm to the position set in the armPosition variable
        robot.armMotor.setPower(1);
        if(armPosition==0){
            robot.armMotor.setTargetPosition(0);
        }else if(armPosition==1){
            robot.armMotor.setTargetPosition(320);
        }else if(armPosition==2){
            robot.armMotor.setTargetPosition(375);
        }else if(armPosition==3){
            robot.armMotor.setTargetPosition(400);
        }else{
            robot.armMotor.setTargetPosition(450);
        }
    }

    public void intakeUpdate() {
        if (intakeOn && gamepad1.x && runtime.time() - lastIntake > 0.5){
            robot.intakeMotor.setPower(0);
            intakeOn = false;
            lastIntake = (double) runtime.time();
        } else if(!carouselRotatorOn && gamepad1.x && runtime.time() - lastIntake > 0.5){
            robot.intakeMotor.setPower(.5);
            intakeOn = true;
            lastIntake = (double) runtime.time();
        }

        if(gamepad1.b){
            robot.intakePusher.setPosition(1);
            sleep(1000);
            robot.intakePusher.setPosition(0);
        }
    }

    public void CarouselRotatorUpdate(){
        if(carouselRotatorOn && gamepad1.y){
            robot.carouselRotator.setPower(0);
            carouselRotatorOn = false;
        } else if(!carouselRotatorOn && gamepad1.y){
            robot.carouselRotator.setPower(.5);
            carouselRotatorOn = true;
        }
    }

    public void Telemetry(){
        telemetry.addData("Arm Position", armPosition);
        telemetry.addData("Encoder", "Count: " + robot.armMotor.getCurrentPosition());
        telemetry.addData("Runtime", runtime.toString());
    }
}
