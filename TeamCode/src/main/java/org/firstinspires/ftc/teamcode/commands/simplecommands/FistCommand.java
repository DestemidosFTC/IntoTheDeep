package org.firstinspires.ftc.teamcode.commands.simplecommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSystem.ArmStage;
import org.firstinspires.ftc.teamcode.subsystems.DestemidosBot;

import java.util.function.DoubleSupplier;

/**
 * A command to drive the robot with joystick input (passed in as {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes.
 */
public class FistCommand extends CommandBase {

    private DestemidosBot destemidosBot;
    private double fistDegrees;


    public FistCommand(DestemidosBot robot, double degrees) {
        destemidosBot = robot;
        fistDegrees = degrees;
        addRequirements( robot.servoSystem );
    }

    @Override
    public void initialize() {
        destemidosBot.servoSystem.fistServoRotation(fistDegrees);
    }

}