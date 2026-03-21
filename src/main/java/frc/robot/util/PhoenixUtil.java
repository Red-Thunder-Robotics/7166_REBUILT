// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import java.util.ArrayDeque;
import java.util.function.Supplier;

public class PhoenixUtil {
    public static final class MotorAction {
        private static final ArrayDeque<MotorAction> queue = new ArrayDeque<>();

        public static void process() {
            final var action = queue.peekFirst();
            if (action == null)
                return;

            if (action.tryRun())
                queue.poll();
        }

        private final String description;
        private final Supplier<StatusCode> action;

        public MotorAction(String description, Supplier<StatusCode> action) {
            this.description = description;
            this.action = action;
        }

        public boolean tryRun() {
            for (int i = 0; i < 5; i++) {
                final var status = action.get();
                if (status.isOK())
                    return true;
            }

            return false;
        }
        public void run() {
            if (!tryRun())
                throw new RuntimeException("FATAL: Failed to " + this.description);
        }
        public void queue() {
            MotorAction.queue.addLast(this);
        }

        public static MotorAction configureMotor(String name, TalonFX motor, TalonFXConfiguration configuration) {
            return new MotorAction("Configure motor '" + name + '\'', () -> motor.getConfigurator().apply(configuration, 0.25d));
        }
        public static MotorAction setMotorPosition(String name, TalonFX motor, double position) {
            return new MotorAction("Set motor '" + name + "' position", () -> motor.setPosition(position, 0.25d));
        }
    }

    public static boolean tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
        for (int i = 0; i < maxAttempts; i++) {
            var error = command.get();
            if (error.isOK()) return true;
        }

        return false;
    }

    public static double getRequestDutyCycle(ControlRequest request) {
        if (request instanceof DutyCycleOut)
            return ((DutyCycleOut) request).Output;

        return 0d;
    }
    public static double getRequestVelocity(ControlRequest request) {
        if (request instanceof MotionMagicVelocityVoltage)
            return ((MotionMagicVelocityVoltage) request).Velocity;

        return 0d;
    }
}
