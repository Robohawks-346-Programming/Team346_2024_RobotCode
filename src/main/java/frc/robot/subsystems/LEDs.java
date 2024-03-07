package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class LEDs extends SubsystemBase {

    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;

    // Rainbow
    private int rainbowFirstPixelHue = 0;

    // Other
    private final Timer flashTimer = new Timer();

    public LEDs() {
        led = new AddressableLED(Constants.LEDConstants.LED_PORT);
        ledBuffer = new AddressableLEDBuffer(Constants.LEDConstants.NUMBER_OF_LEDS);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
    }

    @Override
    public void periodic() {
        if(DriverStation.isDisabled()) {
            rainbow(ledBuffer);
        }
        else {
            if(RobotContainer.shooter.getLaserBreak()) {
                setRGB(ledBuffer, 0, 200, 0);
            }
            else {
                setRGB(ledBuffer, 200, 0, 0);
            }
        }
        led.setData(ledBuffer);
    }

    private void setRGB(AddressableLEDBuffer buffer, int r, int g, int b) {
        for(int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, r, g, b);
        }
    }

    private void setHSV(AddressableLEDBuffer buffer, int h, int s, int v) {
        for(int i = 0; i < buffer.getLength(); i++) {
            buffer.setHSV(i, h, s, v);
        }
    }

    private void setLED(AddressableLEDBuffer buffer, Color color) {
        for(int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, color);
        }
    }

    private void clear(AddressableLEDBuffer buffer) {
        for (int i = 0; i < buffer.getLength(); i++) { 
            setRGB(buffer, 0, 0, 0);
        }
    }

    private void rainbow(AddressableLEDBuffer buffer) {
        for (int i = 0; i < buffer.getLength(); i++) {
            int hue = (rainbowFirstPixelHue + 90 + (i * 180 / buffer.getLength())) % 180;
            setHSV(buffer, hue, 255, 127); 
        }
        rainbowFirstPixelHue = (rainbowFirstPixelHue + 1) % 180;
    }

    private void flashColor(Color color, AddressableLEDBuffer buffer) {
        if ((flashTimer.get()*10%10)%5 < 3) {
            return;
        }
        for (int i = buffer.getLength()/2; i < 3*buffer.getLength()/4; i++) {
            setLED(buffer, color);
        }
    }

    private void setColor(AddressableLEDBuffer buffer, Color color){
        for (int i = 0; i < buffer.getLength(); i++) {
            setLED(buffer, color);
        }
    }
}