package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.DestemidosBot;

public class ArmToLowJunction extends SequentialCommandGroup {

    public ArmToLowJunction(DestemidosBot robot) {
        super(
                //new InstantCommand(()->robot.armSystem.setForearmPosition(ArmSystem.ForearmStage.LOW)),
                //new InstantCommand(()->robot.armSystem.setArmPosition(ArmSystem.ArmStage.LOW))
                //new InstantCommand(()->robot.simpleArm.setPosition(ArmSystem.ArmStage.LOW))
                new InstantCommand(()->robot.simpleArm.goToPosition2(RobotConstants.ARM_LOW_GOAL))
        );
    }
}
