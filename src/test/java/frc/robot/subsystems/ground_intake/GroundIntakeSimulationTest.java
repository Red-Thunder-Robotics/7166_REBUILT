package frc.robot.subsystems.ground_intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.ground_intake.GroundIntakeConstants.*;
import static frc.robot.subsystems.ground_intake.GroundIntakeSubsystem.SystemState.*;
import static org.junit.jupiter.api.Assertions.*;

import java.util.concurrent.atomic.AtomicBoolean;
import org.junit.jupiter.api.*;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.state_machine.IntakeState;
import frc.robot.state_machine.StateMachine;

/** Real subsystem + GroundIntakeIOSim, HAL and a stepped 20 ms simulation clock. */
class GroundIntakeSimulationTest {
    private final AtomicBoolean shooting = new AtomicBoolean();
    private ObservedSim io;
    private GroundIntakeSubsystem intake;

    @BeforeAll static void initializeHal() {
        assertTrue(HAL.initialize(500, 0));
        SimHooks.pauseTiming();
    }
    @BeforeEach void setup() {
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        io = new ObservedSim();
        intake = new GroundIntakeSubsystem(io, shooting::get);
        StateMachine.setIntakeState(IntakeState.HomeOff);
        tick();
    }
    @AfterEach void cleanup() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
    }
    private void tick() { SimHooks.stepTiming(0.02); intake.periodic(); }
    private void settle() { for (int i = 0; i < 100; i++) tick(); }

    @ParameterizedTest @EnumSource(IntakeState.class)
    void everyExistingRequestReachesItsSpecifiedOutputs(IntakeState request) {
        StateMachine.setIntakeState(request);
        settle();
        var expected = switch (request) {
            case HomeOff -> HomeOff;
            case HomeReverse -> HomeReverse;
            case DeployedOff -> DeployedOff;
            case DeployedOn -> DeployedOn;
            case DeployedReverse -> DeployedReverse;
            case OscillateOff -> OscillateOff;
            case OscillateOn -> OscillateOff; // From home, C3 does not start rollers.
        };
        assertEquals(expected, intake.getSystemState());
        double target = (request.isDeployed() ? actuatorPositionDeployed
            : request.isOscillate() ? actuatorPositionOscillate : actuatorPositionHome).in(Inches);
        assertEquals(target, io.inputs.actuatorPositionInches, 0.5);
        double speed = request.areRollersReversed() ? rollerOutputVelocityReverse.in(RotationsPerSecond)
            : request.areRollersPowered() && request != IntakeState.OscillateOn ? rollerOutputVelocity.in(RotationsPerSecond) : 0;
        assertEquals(speed, io.rollerDemand, 1e-9);
    }
    @Test void wantedStateNamesMatchC3() {
        assertArrayEquals(java.util.Arrays.stream(IntakeState.values()).map(Enum::name).toArray(),
            java.util.Arrays.stream(GroundIntakeSubsystem.WantedState.values()).map(Enum::name).toArray());
    }
    @Test void forwardRunsDuringDeploymentAndRepeatedRequestDoesNotInterruptIt() {
        StateMachine.setIntakeState(IntakeState.DeployedOn); tick();
        assertFalse(intake.isAtDeployedPosition());
        assertTrue(io.rollerDemand > 0);
        StateMachine.setIntakeState(IntakeState.DeployedOn); tick();
        assertTrue(io.rollerDemand > 0);
    }
    @Test void oscillateOnStartsForwardFromDeployedAndKeepsItAtMidpoint() {
        StateMachine.setIntakeState(IntakeState.DeployedOff); settle();
        StateMachine.setIntakeState(IntakeState.OscillateOn); tick();
        assertEquals(OscillateOn, intake.getSystemState());
        settle();
        assertEquals(actuatorPositionOscillate.in(Inches), io.inputs.actuatorPositionInches, 0.5);
        assertTrue(io.rollerDemand > 0);
        StateMachine.setIntakeState(IntakeState.OscillateOff); tick();
        assertEquals(0, io.rollerDemand);
    }
    @Test void oscillateOnRetainsReverseWhenNotDeployed() {
        StateMachine.setIntakeState(IntakeState.HomeReverse); tick();
        StateMachine.setIntakeState(IntakeState.OscillateOn); settle();
        assertEquals(OscillateReverse, intake.getSystemState());
        assertTrue(io.rollerDemand < 0);
        assertEquals(actuatorPositionOscillate.in(Inches), io.inputs.actuatorPositionInches, 0.5);
    }
    @Test void stowPullsFuelInUntilHomeThenStops() {
        StateMachine.setIntakeState(IntakeState.DeployedOn); settle();
        StateMachine.setIntakeState(IntakeState.HomeOff); tick();
        assertEquals(HomeOn, intake.getSystemState());
        assertTrue(io.rollerDemand > 0);
        settle();
        assertEquals(HomeOff, intake.getSystemState());
        assertEquals(0, io.rollerDemand);
    }
    @Test void reverseAtHomeWaitsForHomeAndOverridesForward() {
        StateMachine.setIntakeState(IntakeState.DeployedOn); settle();
        StateMachine.setIntakeState(IntakeState.HomeReverse); tick();
        assertEquals(HomeOff, intake.getSystemState());
        assertEquals(0, io.rollerDemand);
        settle();
        assertEquals(HomeReverse, intake.getSystemState());
        assertTrue(io.rollerDemand < 0);
    }
    @Test void interruptedReverseCannotFireLater() {
        StateMachine.setIntakeState(IntakeState.DeployedReverse); tick();
        assertEquals(DeployedOff, intake.getSystemState());
        assertEquals(0, io.rollerDemand);
        StateMachine.setIntakeState(IntakeState.DeployedOff); settle();
        assertEquals(DeployedOff, intake.getSystemState());
        assertEquals(0, io.rollerDemand);
        StateMachine.setIntakeState(IntakeState.DeployedOn); tick();
        assertTrue(io.rollerDemand > 0);
        StateMachine.setIntakeState(IntakeState.DeployedReverse); tick();
        assertTrue(io.rollerDemand < 0);
    }
    @Test void stalledDeployNeverStartsReverseAndCanBeCancelled() {
        io.freeze = true;
        StateMachine.setIntakeState(IntakeState.DeployedReverse); settle();
        assertEquals(DeployedOff, intake.getSystemState());
        assertEquals(0, io.rollerDemand);
        StateMachine.setIntakeState(IntakeState.HomeOff); tick();
        assertEquals(HomeOff, intake.getSystemState());
    }
    @Test void disableStopsMovementAndReenableDoesNotRestartRollers() {
        StateMachine.setIntakeState(IntakeState.DeployedOn); tick();
        DriverStationSim.setEnabled(false); DriverStationSim.notifyNewData(); tick();
        assertEquals(Disabled, intake.getSystemState());
        assertEquals(0, io.rollerDemand);
        double stoppedPosition = io.inputs.actuatorPositionInches;
        settle();
        assertEquals(stoppedPosition, io.inputs.actuatorPositionInches, 1e-9);
        assertEquals(IntakeState.HomeOff, StateMachine.getIntakeState());
        DriverStationSim.setEnabled(true); DriverStationSim.notifyNewData(); tick();
        assertEquals(0, io.rollerDemand);
        settle(); assertEquals(HomeOff, intake.getSystemState());
    }
    @Test void zeroingOwnsActuatorAndStopsRollers() {
        StateMachine.setIntakeState(IntakeState.DeployedOn); tick();
        var zero = intake.zeroMechanisms(false);
        CommandScheduler.getInstance().schedule(zero);
        int writes = io.positionWrites;
        tick();
        assertEquals(Zeroing, intake.getSystemState());
        assertEquals(writes, io.positionWrites);
        assertEquals(0, io.rollerDemand);
        zero.cancel(); tick(); assertEquals(0, io.rollerDemand);
    }
    @ParameterizedTest @EnumSource(Mode.class)
    void shootingPushRetractsAfterModeSpecificDelay(Mode mode) {
        DriverStationSim.setAutonomous(mode == Mode.AUTO); DriverStationSim.notifyNewData();
        StateMachine.setIntakeState(IntakeState.DeployedOff); settle();
        shooting.set(true); tick(); tick();
        assertEquals(DeployedOn, intake.getSystemState());
        for (int i = 0; i < 8; i++) tick();
        assertEquals(DeployedOn, intake.getSystemState());
        for (int i = 0; i < 11; i++) tick(); // 0.4 seconds after timer start.
        assertEquals(mode == Mode.AUTO ? IntakeState.HomeOff : IntakeState.DeployedOn,
            StateMachine.getIntakeState());
        for (int i = 0; i < 10; i++) tick();
        assertEquals(IntakeState.HomeOff, StateMachine.getIntakeState());
        settle(); assertEquals(HomeOff, intake.getSystemState()); assertEquals(0, io.rollerDemand);
        shooting.set(false); tick();
        StateMachine.setIntakeState(IntakeState.DeployedOff); settle();
        assertEquals(DeployedOff, intake.getSystemState());
    }
    enum Mode { TELEOP, AUTO }
    private static class ObservedSim implements GroundIntakeIO {
        final GroundIntakeIOSim sim = new GroundIntakeIOSim();
        GroundIntakeIOInputs inputs;
        double rollerDemand;
        int positionWrites;
        boolean freeze;
        public void updateInputs(GroundIntakeIOInputs inputs) {
            if (!freeze) sim.updateInputs(inputs);
            this.inputs = inputs;
        }
        public void setActuatorPosition(Distance position) { positionWrites++; sim.setActuatorPosition(position); }
        public void actuatorStop() { sim.actuatorStop(); }
        public void rollerVelocity(AngularVelocity velocity) {
            rollerDemand = velocity.in(RotationsPerSecond); sim.rollerVelocity(velocity);
        }
        public void rollerStop() { rollerDemand = 0; sim.rollerStop(); }
    }
}
