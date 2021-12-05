package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Controls the robot's mecanum drive base
 * @author John Brereton
 * @since 2021-10-08
 */

public class RRBotMecanumDrive
{
    RRBotHardware robot;

    private ElapsedTime autoMoveTime = new ElapsedTime();

    private boolean isAutoMove = false;
    private double autoTime;

    /**
     * Constructor gets hardware object from teleop class
     * @param robot contains the hardware elements of the robot
     */
    public RRBotMecanumDrive(RRBotHardware robot)
    {
        this.robot = robot;
    }

    /**
     * Calculates the velocity values for the drive motors in a mecanum configuration.
     * @param leftX X position of left joystick
     * @param leftY Y position of left joystick
     * @param rightX X position of right joystick
     * @param rightY Y position of right joystick
     * @param doFunction whether a function should be used on the input values
     * @return velocities - array of motor velocities
     */
    public double[] calcVelocities(double leftX, double leftY, double rightX, double rightY, boolean doFunction)
    {
        double moveX = rightX;
        double moveY1 = leftY;
        double turn = leftX;
        double moveY2 = rightY;

        //remap input values using a function
        if(doFunction)
        {
            moveX = inputFunction(moveX);
            moveY1 = inputFunction(moveY1);
            turn = inputFunction(turn);
            moveY2 = inputFunction(moveY2);
        }

        double v1 = moveY1 + moveX + turn + moveY2;
        double v2 = moveY1 - moveX - turn + moveY2;
        double v3 = moveY1 + moveX - turn + moveY2;
        double v4 = moveY1 - moveX + turn + moveY2;

        double max = Math.abs(v1);
        if(Math.abs(v2) > max)
            max = Math.abs(v2);
        if(Math.abs(v3) > max)
            max = Math.abs(v3);
        if(Math.abs(v4) > max)
            max = Math.abs(v4);
        if(max > 1)
        {
            v1 /= max;
            v2 /= max;
            v3 /= max;
            v4 /= max;
        }

        double[] velocities = {v1, v2, v3, v4};
        return velocities;
    }

    /**
     * Sets the motor power for manual drive. The parameters are sent to calcVelocities.
     * @param leftX X position of left joystick
     * @param leftY Y position of left joystick
     * @param rightX X position of right joystick
     * @param rightY Y position of right joystick
     * @param doFunction whether a function should be used on the input values
     */
    public void setMotorPower(double leftX, double leftY, double rightX, double rightY, boolean doFunction)
    {
        //calculate the velocities
        double[] velocities = calcVelocities(leftX, leftY, rightX, rightY, doFunction);

        //set the motor power
        robot.frontLeftDrive.setPower(velocities[0]);
        robot.frontRightDrive.setPower(velocities[1]);
        robot.rearLeftDrive.setPower(velocities[2]);
        robot.rearRightDrive.setPower(velocities[3]);
    }

    /**
     * Function to be executed on the joystick input values. Not currently used
     * @param input joystick value
     * @return value - input value scaled by function
     */
    public double inputFunction(double input)
    {
        //double value = input;

        //f(x) = x^3
        //value = value * value * value;
        //return value;

        /*if(value > 0)
        {
            Range.clip(value, driveDeadBand, 1);
        }
        else if(value < 0)
        {
            Range.clip(value, -1, -driveDeadBand);
        }*/
        /*if(Math.abs(value - 0) < 0.01)
        {
            value = 0;
        }*/

        /*if(input >= 0)
        {
            value = Math.sqrt(input);
        }
        else
        {
            value = -Math.sqrt(-input);
        }
        return value;*/

        //f(x) = x^2
        double value = input * input;

        if(input < 0)
        {
            return value * -1;
        }
        else
        {
            return value;
        }
    }

    /**
     * Automatically moves the robot based on a set speed and time. Meant to be used in teleop.
     * @param speed speed of movement
     * @param time how long the movement should take
     */
    public void AutoMove(double speed, double time)
    {
        isAutoMove = true;
        autoTime = time;

        autoMoveTime.reset();

        robot.rearRightDrive.setPower(speed);
        robot.rearLeftDrive.setPower(speed);
        robot.frontRightDrive.setPower(speed);
        robot.frontLeftDrive.setPower(speed);
    }
    /**
     * Automatically moves the robot based on a set speed, inches, and time
     * @param speed
     */
    public void AutoEncoderMove(double speed, double inches, double time)
    {

    }
    /**
     * Checks if the movement is done by comparing the time elapsed and the time the movement is set to take.
     */
    public void AutoMoveEndCheck()
    {
        if(autoMoveTime.milliseconds() >= autoTime)
        {
            isAutoMove = false;
        }
    }

    /**
     * Returns whether an auto move is currently occurring
     * @return isAutoMove
     */
    public boolean getIsAutoMove()
    {
        return isAutoMove;
    }
}
