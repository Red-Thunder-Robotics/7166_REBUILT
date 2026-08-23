package frc.robot.subsystems.light_emitting_diodes;

import static frc.robot.subsystems.light_emitting_diodes.LightEmittingDiodesConstants.*;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.StripTypeValue;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public final class LightEmittingDiodesIOReal implements LightEmittingDiodesIO {
    private final CANdle m_candle = new CANdle(candleID, Constants.CANBUS);

    private final SolidColor m_solidColorRequest = new SolidColor(0, ledCount);
    private final StrobeAnimation m_strobeRequest = new StrobeAnimation(0, ledCount).withFrameRate(animationSpeed);

    public LightEmittingDiodesIOReal() {
        var config = new CANdleConfiguration();

        config.LED.BrightnessScalar = brightness;
        config.LED.StripType = StripTypeValue.GRB;

        PhoenixUtil.tryUntilOk(5, () -> m_candle.getConfigurator().apply(config));
    }

    @Override
    public void solidWhite() {
        m_candle.setControl(m_solidColorRequest.withColor(colorWhite));
    }

    @Override
    public void solidRed() {
        m_candle.setControl(m_solidColorRequest.withColor(colorRed));
    }

    @Override
    public void solidGreen() {
        m_candle.setControl(m_solidColorRequest.withColor(colorGreen));
    }

    @Override
    public void flashingGreen() {
        m_candle.setControl(m_strobeRequest.withColor(colorGreen));
    }

    @Override
    public void solidBlue() {
        m_candle.setControl(m_solidColorRequest.withColor(colorBlue));
    }

    @Override
    public void flashingBlue() {
        m_candle.setControl(m_strobeRequest.withColor(colorBlue));
    }
}
