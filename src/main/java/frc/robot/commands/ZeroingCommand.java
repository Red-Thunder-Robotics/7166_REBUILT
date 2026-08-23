package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.BooleanSupplier;

public final class ZeroingCommand extends Command {
    public static boolean isSubsystemZeroing(Subsystem subsystem) {
        var currentCommand = subsystem.getCurrentCommand();
        return currentCommand != null && currentCommand instanceof ZeroingCommand;
    }

    private Timer m_initialTimer = new Timer();
    private Timer m_zeroTimer = new Timer();
    private boolean m_finished;

    private final Runnable m_drive;
    private final BooleanSupplier m_canZero;
    private final Runnable m_stop;
    private final Runnable m_zero;

    public ZeroingCommand(Subsystem subsystem, Runnable drive, BooleanSupplier canZero, Runnable stop, Runnable zero) {
        addRequirements(subsystem);

        m_drive = drive;
        m_canZero = canZero;
        m_stop = stop;
        m_zero = zero;
    }

    @Override
    public void initialize() {
        m_zeroTimer.stop();
        m_initialTimer.restart();
        m_finished = false;
    }

    @Override
    public void execute() {
        m_drive.run();

        if (!m_initialTimer.hasElapsed(0.5d)) return;

        final boolean timerWasRunning = m_zeroTimer.isRunning();

        if (m_canZero.getAsBoolean()) {
            if (!timerWasRunning) m_zeroTimer.restart();
            else if (m_zeroTimer.hasElapsed(0.25d)) m_finished = true;
        } else if (timerWasRunning) m_zeroTimer.stop();
    }

    @Override
    public void end(boolean interrupted) {
        m_stop.run();
        if (!interrupted) m_zero.run();
    }

    @Override
    public boolean isFinished() {
        return m_finished;
    }
}
