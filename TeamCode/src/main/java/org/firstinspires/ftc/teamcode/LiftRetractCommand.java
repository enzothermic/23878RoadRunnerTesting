package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.BooleanSupplier;

public class LiftRetractCommand extends CommandBase {

    private final LiftSubsystem liftSubsystem;
    private final BooleanSupplier overrideLimits, leftDown, rightDown;
    public LiftRetractCommand(LiftSubsystem liftSubsystem, BooleanSupplier overrideLimits, BooleanSupplier leftDown, BooleanSupplier rightDown) {
        this.liftSubsystem = liftSubsystem;
        this.overrideLimits = overrideLimits;
        this.leftDown = leftDown;
        this.rightDown = rightDown;
    }

    @Override
    public void execute(){
        liftSubsystem.retract(overrideLimits.getAsBoolean(), leftDown.getAsBoolean(), rightDown.getAsBoolean());
    }

    @Override
    public void end(boolean interrupted){
        liftSubsystem.stop();
    }
}
