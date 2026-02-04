package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private static LEDSubsystem INSTANCE;

    public static final int NUM_LEDS = 107;

    public static LEDSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new LEDSubsystem();
        return INSTANCE;
    }
    
    private CANdle candle = new CANdle(40, "CantDrive");
    private SolidColor solidColorRequest = new SolidColor(0, 8).withColor(RGBWColor.fromHSV(0, 1, 1));

    public LEDSubsystem() {
        configureCANdle();
        setRange(0, NUM_LEDS);
        setColor(0, 1, 1, 0);
        applyControl();
        setRange(0, 0);
    }
    private void configureCANdle() {
        CANdleConfiguration candleConfig = new CANdleConfiguration();
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
     * @param bright Brightness, range of: { 0-1 }
     */
    public void setColor(double h, double s, double v, double bright) {
        h = MathUtil.clamp(h, 0, 360);
        s = MathUtil.clamp(s, 0, 1);
        v = MathUtil.clamp(v, 0, 1);
        solidColorRequest.Color = RGBWColor.fromHSV(h, s, v).scaleBrightness(bright);
    }
    /**
     * Changes the color that the CANdle sets the LEDs to.
     * @param h Hue, range of: { 0-360 }
     * @param s Saturation, range of: { 0-1 }
     * @param v Value, range of: { 0-1 }
     */
    public void setColor(double h, double s, double v) {
        h = MathUtil.clamp(h, 0, 360);
        s = MathUtil.clamp(s, 0, 1);
        v = MathUtil.clamp(v, 0, 1);
        solidColorRequest.Color = RGBWColor.fromHSV(h, s, v);
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
        if (index < 0) index = 0; if (index > NUM_LEDS) index = NUM_LEDS;
        solidColorRequest.LEDStartIndex = index;
    }
    public void setEnd(int index) {
        if (index < 0) index = 0; if (index > NUM_LEDS) index = NUM_LEDS;
        solidColorRequest.LEDEndIndex = index;
    }

    public void setRange(int startIndex, int endIndex) {
        setStart(startIndex);
        setEnd(endIndex);
    }
}
