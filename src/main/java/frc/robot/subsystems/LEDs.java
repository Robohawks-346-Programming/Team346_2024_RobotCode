package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.Constants.LEDConstants;

public class LEDs extends SubsystemBase {

    // LEDs
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    // Rainbow
    private int rainbowFirstPixelHue = 0;

    // Other
    private final Timer flashTimer = new Timer();
    private boolean off = false;

    public LEDs() {

        led = new AddressableLED(Constants.LEDConstants.LED_PORT);
        ledBuffer = new AddressableLEDBuffer(Constants.LEDConstants.NUMBER_OF_LEDS);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();

        flashTimer.reset();
        flashTimer.start();

    }

    @Override
    public void periodic() {

        clear();
        
        if (!off) {
            if (DriverStation.isDisabled()){
                rainbow();
            }
        }

        led.setData(ledBuffer);

    }

    private void setRGB(int i, int r, int g, int b) {
        ledBuffer.setRGB(i, r, g, b);
    }

    private void setHSV(int i, int h, int s, int v) {
        ledBuffer.setHSV(i, h, s, v);
    }

    private void setLED(int i, Color color) {
        ledBuffer.setLED(i, color);
    }

    private void clear() {
        for (int i = 0; i < Constants.LEDConstants.NUMBER_OF_LEDS; i++) { 
            setRGB(i, 0, 0, 0);
        }
    }

    private void rainbow() {
        for (int i = 0; i < Constants.LEDConstants.NUMBER_OF_LEDS; i++) {
            int hue = (rainbowFirstPixelHue + 90 + (i * 180 / Constants.LEDConstants.NUMBER_OF_LEDS)) % 180;
            setHSV(i, hue, 255, 127); 
        }
            rainbowFirstPixelHue = (rainbowFirstPixelHue + 1) % 180;
    }

    private void flashColor(Color color) {
        if ((flashTimer.get()*10%10)%5 < 3) {
            return;
        }

        for (int i = Constants.LEDConstants.NUMBER_OF_LEDS/2; i < 3*Constants.LEDConstants.NUMBER_OF_LEDS/4; i++) {
            setLED(i, color);
        }
        
    }

    private void flashOnWhistle(Color color1) {

        if (DriverStation.isTeleopEnabled() && Math.abs(DriverStation.getMatchTime()-30) < 1.5) {
            Color color = (flashTimer.get()* 10 % 10) % 5 < 2.5 ? Color.kBlack : color1;
            for (int i = 0; i < Constants.LEDConstants.NUMBER_OF_LEDS; i++) {
                setLED(i, color);
            }
        }

    }

    private void flashOnIntake() {
        if (RobotState.getInstance().isIntaking()) {
            Color color = (RobotState.getInstance().hasIntaked() && (flashTimer.get()*10%10)%5 < 2.5) ? Color.kBlack : Color.kOrange;
            for (int i = 0; i < Constants.LEDConstants.NUMBER_OF_LEDS / 4; i++) {
                setLED(i, color);
            }
            for (int i = 3 * Constants.LEDConstants.NUMBER_OF_LEDS / 4; i < Constants.LEDConstants.NUMBER_OF_LEDS; i++) {
                setLED(i, color);
            }
        }
    }

    public void setOff(boolean offf) {
        off = offf;
    }
    
}