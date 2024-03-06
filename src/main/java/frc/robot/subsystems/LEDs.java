package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.LEDConstants;

public class LEDs extends SubsystemBase {

        private final AddressableLED ledFR = new AddressableLED(Constants.LEDConstants.LED_FR_PORT);
        private final AddressableLED ledFL = new AddressableLED(Constants.LEDConstants.LED_FL_PORT);
        //private final AddressableLED ledBR = new AddressableLED(Constants.LEDConstants.LED_BR_PORT);
        // private final AddressableLED ledBL = new AddressableLED(Constants.LEDConstants.LED_BL_PORT);


        private final AddressableLEDBuffer ledBufferFR = new AddressableLEDBuffer(Constants.LEDConstants.NUMBER_OF_LEDS_FR);
        private final AddressableLEDBuffer ledBufferFL = new AddressableLEDBuffer(Constants.LEDConstants.NUMBER_OF_LEDS_FL);
        private final AddressableLEDBuffer ledBufferBR = new AddressableLEDBuffer(Constants.LEDConstants.NUMBER_OF_LEDS_BR);
        private final AddressableLEDBuffer ledBufferBL = new AddressableLEDBuffer(Constants.LEDConstants.NUMBER_OF_LEDS_BL);

    private final AddressableLEDBuffer[] buffers = {ledBufferBR, ledBufferFR, ledBufferFL, ledBufferBL};
    //private final AddressableLED[] leds = {ledFR, ledFL, ledBR, ledBL};


    // Rainbow
    private int rainbowFirstPixelHue = 0;

    // Other
    private final Timer flashTimer = new Timer();

    public LEDs() {

        ledFR.setLength(ledBufferFR.getLength());
        ledFL.setLength(ledBufferFL.getLength());
        //ledBR.setLength(ledBufferBR.getLength());
        // ledBL.setLength(ledBufferBL.getLength());

        ledFR.setData(ledBufferFR);
        ledFL.setData(ledBufferFL);
        //ledBR.setData(ledBufferBR);
        // ledBL.setData(ledBufferBL);

        ledFR.start();
        ledFL.start();
        //ledBR.start();
        // ledBL.start();

        flashTimer.reset();
        flashTimer.start();

    }

    @Override
    public void periodic() {
            // if (DriverStation.isDisabled()){
            //     for (AddressableLEDBuffer buffer: buffers){
            //         rainbow(buffer);
            //     }
            // } else if (RobotContainer.intake.isIntaking){
            //     for (AddressableLEDBuffer buffer: buffers){
            //         flashColor(Color.kIndianRed, buffer);
            //     }
            // } else if (RobotContainer.intake.hasGamePiece){
            //     for (AddressableLEDBuffer buffer: buffers){
            //         flashColor(Color.kSeaGreen, buffer);
            //     }
            // } else if (RobotContainer.indexer.storingGamePiece){
            //     for (AddressableLEDBuffer buffer: buffers){
            //         flashColor(Color.kOrange, buffer);
            //     }
            // } else {
            //     for (AddressableLEDBuffer buffer: buffers){
            //         setLED(buffer, rainbowFirstPixelHue, Color.kAliceBlue);
            //     }
            // }
            
            setColor(ledBufferFR, Color.kAquamarine);
            setColor(ledBufferBR, Color.kAquamarine);
        ledFR.setData(ledBufferFR);
        ledFL.setData(ledBufferFL);
        //ledBR.setData(ledBufferBR);
        // ledBL.setData(ledBufferBL);
    }

    private void setRGB(AddressableLEDBuffer buffer, int i, int r, int g, int b) {
        buffer.setRGB(i, r, g, b);
    }

    private void setHSV(AddressableLEDBuffer buffer, int i, int h, int s, int v) {
        buffer.setHSV(i, h, s, v);
    }

    private void setLED(AddressableLEDBuffer buffer, int i, Color color) {
        buffer.setLED(i, color);
    }

    private void clear(AddressableLEDBuffer buffer) {
        for (int i = 0; i < buffer.getLength(); i++) { 
            setRGB(buffer, i, 0, 0, 0);
        }
    }

    private void rainbow(AddressableLEDBuffer buffer) {
        for (int i = 0; i < buffer.getLength(); i++) {
            int hue = (rainbowFirstPixelHue + 90 + (i * 180 / buffer.getLength())) % 180;
            setHSV(buffer, i, hue, 255, 127); 
        }
        rainbowFirstPixelHue = (rainbowFirstPixelHue + 1) % 180;
    }

    private void flashColor(Color color, AddressableLEDBuffer buffer) {
        if ((flashTimer.get()*10%10)%5 < 3) {
            return;
        }
        for (int i = buffer.getLength()/2; i < 3*buffer.getLength()/4; i++) {
            setLED(buffer, i, color);
        }
    }

    private void setColor(AddressableLEDBuffer buffer, Color color){
        for (int i = 0; i < buffer.getLength(); i++) {
            setLED(buffer, i, color);
        }
    }
}