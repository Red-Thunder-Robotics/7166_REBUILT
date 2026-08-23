package frc.robot.subsystems.light_emitting_diodes;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.units.measure.Frequency;

public final class LightEmittingDiodesConstants {
    public static final int candleID = -1;
    public static final int ledCount = 40;

    public static final double brightness = 1d;
    public static final Frequency animationSpeed = Hertz.of(20d);

    public static final int primaryHue = 255;
    public static final int secondaryHue = 0;
    public static final int tertiaryHue = 170;

    public static final RGBWColor colorWhite = new RGBWColor(primaryHue, primaryHue, primaryHue);
    public static final RGBWColor colorRed = new RGBWColor(primaryHue, secondaryHue, secondaryHue);
    public static final RGBWColor colorGreen = new RGBWColor(secondaryHue, primaryHue, secondaryHue);
    public static final RGBWColor colorBlue = new RGBWColor(primaryHue, primaryHue, secondaryHue);
}
