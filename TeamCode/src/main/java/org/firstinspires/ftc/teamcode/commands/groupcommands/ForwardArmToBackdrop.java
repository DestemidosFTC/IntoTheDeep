package org.firstinspires.ftc.teamcode.commands.groupcommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.commands.simplecommands.ArmCommand;
import org.firstinspires.ftc.teamcode.commands.simplecommands.FistCommand;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.ArmSystem.ArmStage;
import org.firstinspires.ftc.teamcode.subsystems.DestemidosBot;


public class ForwardArmToBackdrop extends ParallelCommandGroup{

    public ForwardArmToBackdrop(DestemidosBot robot) {
        addCommands(
                new ArmCommand(robot, 75 ),
                new FistCommand(robot, 100)
        );
        addRequirements( robot.servoSystem, robot.armSystem);
    }

}