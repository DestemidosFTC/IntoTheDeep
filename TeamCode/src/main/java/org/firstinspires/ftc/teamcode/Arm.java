package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * Arm.java: PIDF Controlled Arm subsystem.
 *
 * This implements a PIDF Controlled Arm subsystem. It provides control of an arm in either Autonomous or TeleOp
 * OpMode. The core code of controlling the arm is in the armUpdate method. This method needs to be called periodically
 * so that it can use the PIDF controller to control and maintain the arm position. It provides the setPosition
 * method to set the arm's target position with a power limit that determines the max speed of the arm movement. The
 * control supports gravity compensation where it will apply appropriate power to hold the arm against gravity making
 * it gravity neutral (i.e. the arm will hold its position at any angle). It also provide the isOnTarget method for
 * autonomous code to determine if the arm has reached its target position so that it can perform the next operation.
 * The setPower method allows TeleOp code to use the joystick of a game controller to control the arm going up and
 * down with variable speed. Unlike other arm control implementations where they operate in the units of encoder
 * ticks, this implementation uses real world units of arm angle in degrees. Therefore, instead of calling setPosition
 * to move the arm to position 3225 encoder ticks, we call setPosition to move the arm to 75 degrees from its zero
 * position which is vertical down (90-degree being horizontal). It is much more intuitive to call setPosition(75) to
 * set arm position to 75 degree since this code will translate the real world degrees back to encoder ticks when
 * performing the control.
 */

public class Arm
{
    // Arm characterization constants.

    // KP specifies the Proportional PID coefficient in PIDF.
    private static final double KP = 0.3;

    // KI specifies the Integral PID coefficient in PIDF.
    private static final double KI = 0.0;

    // KD specifies the Derivative PID coefficient in PIDF.
    private static final double KD = 0.0;

    // KF specifies the arm holding power when the arm is at 90-degree (horizontal).
    private static final double KF = 0.025;

    // ZERO_OFFSET specifies the arm angle in degrees from vertical when it is at resting position.
    private static final double ZERO_OFFSET = 55;

    // MIN_POS specifies the arm angle in degrees at its minimum position (usually the same as ZERO_OFFSET).
    private static final double MIN_POS = ZERO_OFFSET;

    // MAX_POS specifies the arm angle in degrees at its maximum position.
    private static final double MAX_POS = 205;

    // TICKS_PER_DEGREE specifies the scaling factor to translate arm angle in degrees to encoder ticks.
    private static final double ENCODER_CPR = 288.0;
    private static final double GEAR_RATIO = 1;	// don't include motor internal gear ratio
    private static final double TICKS_PER_DEGREE = ENCODER_CPR * GEAR_RATIO / 360.0;

    // Arm subsystem components.
    private final DcMotorEx leftArmMotor;
    private final DcMotorEx rightArmMotor;
    private final PIDController pidController;

    // Arm states.
    private double targetPosInDegrees;
    private double revPowerLimit, fwdPowerLimit;

    /**
     * Constructor: create and initialize everything required by the arm subsystem including the arm motor and
     * PID controller.
     *
     */
    public Arm(HardwareMap hardwareMap)
    {
        // Create arm motor and initialize it.
        leftArmMotor = hardwareMap.get(DcMotorEx.class, "fore");
        rightArmMotor = hardwareMap.get(DcMotorEx.class, "forearm");

        leftArmMotor.setDirection(DcMotor.Direction.FORWARD);
        rightArmMotor.setDirection(DcMotor.Direction.REVERSE);

        leftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // You may want to use RUN_USING_ENCODER if you want to do velocity control instead of OpenLoop.
        leftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Create PID controller for the arm and initialize it with proper PID coefficients.
        pidController = new PIDController(KP, KI, KD);

        this.targetPosInDegrees = ZERO_OFFSET;
        this.revPowerLimit = -1.0;
        this.fwdPowerLimit = 1.0;
    }

    /**
     * This method must be called periodically so that it will do PID control of the arm towards set target position
     * and hold it. Note: this is the equivalent of the .update() method in Command-Based (e.g. FTCLib).
     */
    public void armUpdate()
    {
        // targetPosInDegrees, revPowerLimit and fwdPowerLimit are set by the setPosition method.
        double targetPosInTicks = (targetPosInDegrees - ZERO_OFFSET) * TICKS_PER_DEGREE;
        double currPosInTicks = leftArmMotor.getCurrentPosition();
        double pidOutput = pidController.calculate(currPosInTicks, targetPosInTicks);
        // Calculate gravity compensation (assuming arm at horizontal position is 90-degree).
        double gravityComp = KF * Math.sin(Math.toRadians(ticksToRealWorldDegrees(currPosInTicks)));
        double power = pidOutput + gravityComp;
        // Clip power to the range of revPowerLimit and fwdPowerLimit.
        // For an arm, revPowerLimit and fwdPowerLimit should be symmetrical (in the range of -powerLimit and +powerLimit).
        power = Range.clip(power, revPowerLimit, fwdPowerLimit);
        leftArmMotor.setPower(power);
        rightArmMotor.setPower(power);
    }

    /**
     * This method is typically called by autonomous to determine when the arm has reached target so that it can move
     * on to perform the next operation.
     *
     * @param toleranceInDegrees specifies the tolerance in degrees within which we will consider reaching target.
     * @return true if arm is on target within tolerance, false otherwise.
     */
    public boolean isOnTarget(double toleranceInDegrees)
    {
        double currPosInDegrees = getPosition();
        return Math.abs(targetPosInDegrees - currPosInDegrees) <= toleranceInDegrees;
    }

    /**
     * This method can be called by autonomous or teleop to set the arm target. Typically, in TeleOp, one can react to
     * a button press and call this method to move the arm to a preset position.
     *
     * @param targetPosInDegrees specifies the target position in degrees.
     * @param powerLimit specifies the maximum power for the arm movement.
     */
    public void setPosition(double targetPosInDegrees, double powerLimit)
    {
        this.targetPosInDegrees = targetPosInDegrees;
        powerLimit = Math.abs(powerLimit);
        this.revPowerLimit = -powerLimit;
        this.fwdPowerLimit = powerLimit;
    }

    /**
     * This method is typically used by TeleOp to control the arm movement by the value of the joystick so that the
     * speed of the arm movement can be controlled by the joystick. Since this is PID controlled, the arm will slow
     * down when approaching the min or max limits even though the joystick is pushed to max position.
     *
     * @param power specifies the maximum power for the arm movement.
     */
    public void setPower(double power)
    {
        if (power > 0.0)
        {
            // Move arm towards max position with specified power.
            setPosition(MAX_POS, power);
        }
        else if (power < 0.0)
        {
            // Move arm towards min position with specified power.
            setPosition(MIN_POS, power);
        }
        else
        {
            // Hold arm position without power limit.
            setPosition(getPosition(), 1.0);
        }
    }

    /**
     * This method translates encoder ticks to real world arm position in degrees.
     *
     * @param encoderTicks specifies the motor encoder ticks.
     * @return translated arm position in degrees.
     */
    public double ticksToRealWorldDegrees(double encoderTicks)
    {
        return encoderTicks / TICKS_PER_DEGREE + ZERO_OFFSET;
    }

    /**
     * This method returns the arm position in real world degrees.
     *
     * @return arm position in degrees.
     */
    public double getPosition()
    {
        return ticksToRealWorldDegrees(leftArmMotor.getCurrentPosition());
    }
}