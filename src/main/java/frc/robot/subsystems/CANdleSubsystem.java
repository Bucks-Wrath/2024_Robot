package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeviceIds;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

public class CANdleSubsystem extends SubsystemBase {

    private final CANdle m_candle = new CANdle(DeviceIds.CANdle.CANdleId);
    private final int LedCount = 400;
    public String color;


    private Animation m_toAnimate = null;

    public CANdleSubsystem() {
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = 0.8;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configAllSettings(configAll, 100);
    }

    /* Wrappers so we can access the CANdle from the subsystem */
    public double getVbat() { return m_candle.getBusVoltage(); }
    public double get5V() { return m_candle.get5VRailVoltage(); }
    public double getCurrent() { return m_candle.getCurrent(); }
    public double getTemperature() { return m_candle.getTemperature(); }
    public void configBrightness(double percent) { m_candle.configBrightnessScalar(percent, 0); }
    public void configLos(boolean disableWhenLos) { m_candle.configLOSBehavior(disableWhenLos, 0); }
    public void configLedType(LEDStripType type) { m_candle.configLEDType(type, 0); }
    public void configStatusLedBehavior(boolean offWhenActive) { m_candle.configStatusLedState(offWhenActive, 0); }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
         m_candle.animate(m_toAnimate);
    }

    public void setAnimate(String color){
        this.color = color;
        switch(color) {
            case "Orange":
                m_toAnimate = new SingleFadeAnimation(255, 165, 0, 0, 0.7, LedCount);
                break;
            case "Purple":
                m_toAnimate = new SingleFadeAnimation(128, 0, 128, 0, 0.7, LedCount);
                break;
            case "Strobe Orange":
                m_toAnimate = new StrobeAnimation(255, 165, 0, 0, 0.4, LedCount);
                break;
            case "Rainbow":
                m_toAnimate = new RainbowAnimation(0.8, 0.7, LedCount);
            }   
    }
}
