package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private static LEDSubsystem INSTANCE;

    public static final int HIGHEST_INDEX = 107;
    public static final RGBWColor EMPTY_COLOR = new RGBWColor(0, 0, 0);

    public static LEDSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new LEDSubsystem();
        return INSTANCE;
    }
    
    private CANdle candle = new CANdle(6, "CantShoot");
    private SolidColor solidColorRequest = new SolidColor(0, 8).withColor(EMPTY_COLOR);

    public LEDSubsystem() {
        configureCANdle();
        setRange(0, HIGHEST_INDEX);
        setColor(EMPTY_COLOR);
        applyControl();
        setRange(0, 0);
    }
    private void configureCANdle() {
        CANdleConfiguration candleConfig = new CANdleConfiguration();
        candleConfig.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
        candleConfig.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.DisableLEDs;
        candleConfig.LED.StripType = StripTypeValue.RGB;

        candle.getConfigurator().apply(candleConfig);
    }

    @Override
    public void periodic() {
        applyControl();
    }

    public void applyControl() {
        candle.setControl(solidColorRequest);
    }

    /**
     * Changes the color that the CANdle sets the LEDs to.
     * @param h Hue, range of: { 0-360 }
     * @param s Saturation, range of: { 0-1 }
     * @param v Value, range of: { 0-1 }
     * @param brightness Brightness, range of: { 0-1 }
     */
    public void setColor(double h, double s, double v, double brightness) {
        h = MathUtil.clamp(h, 0, 360);
        s = MathUtil.clamp(s, 0, 1);
        v = MathUtil.clamp(v, 0, 1);
        solidColorRequest.Color = RGBWColor.fromHSV(h, s, v).scaleBrightness(brightness);
    }
    public void setColor(RGBWColor color) {
        solidColorRequest.Color = color;
    }
    public RGBWColor getColor() {
        return solidColorRequest.Color;
    }

    public int getStart() {
        return solidColorRequest.LEDStartIndex;
    }
    public int getEnd() {
        return solidColorRequest.LEDEndIndex;
    }
    public void setStart(int index) {
        if (index < 0) index = 0; if (index > HIGHEST_INDEX) index = HIGHEST_INDEX;
        solidColorRequest.LEDStartIndex = index;
    }
    public void setEnd(int index) {
        if (index < 0) index = 0; if (index > HIGHEST_INDEX) index = HIGHEST_INDEX;
        solidColorRequest.LEDEndIndex = index;
    }

    public void setRange(int startIndex, int endIndex) {
        setStart(startIndex);
        setEnd(endIndex);
    }
}
