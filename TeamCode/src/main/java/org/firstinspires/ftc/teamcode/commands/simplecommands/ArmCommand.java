package org.firstinspires.ftc.teamcode.commands.simplecommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSystem.ArmStage;
import org.firstinspires.ftc.teamcode.subsystems.DestemidosBot;



public class ArmCommand extends CommandBase {

    private DestemidosBot destemidosBot;
    private int position;


    public ArmCommand(DestemidosBot robot, int target) {
        destemidosBot = robot;
        position = target;
        addRequirements( destemidosBot.armSystem );
    }


    @Override
    public void execute() {
        destemidosBot.armSystem.PIDcontroller(position);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}