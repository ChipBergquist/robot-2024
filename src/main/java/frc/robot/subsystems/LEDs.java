package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LEDConstants;

public class LEDs extends SubsystemBase {
    private final AddressableLED led;

    private final AddressableLEDBuffer ledBuffer;

    private double lastAnimUpdate = 0;

    private Constants.LEDStates state = Constants.LEDStates.nothing;
    private Boolean lightsOn = true;
    private int rainbowH = 0;
    private final int rainbowS = 150;
    private final int rainbowV = 200;

    /**
     * constructs the leds class
     */
    public LEDs() {
        // create an addressable led object and a buffer for it
        led = new AddressableLED(LEDConstants.port);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.length);

        // set the length of the strip and give it data
        led.setLength(LEDConstants.length);
        led.setData(ledBuffer);
        led.start();
    }

    @Override
    public void periodic() {
        updateAnimation();

        led.setData(ledBuffer);
    }

    /**
     * set the color of the entire strip to a rgb color
     */
    public void setColorRGB(int r, int g, int b) {
        // loop through the buffer and set each led to the desired color
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, r, g, b);
        }
    }

    public void setColorHSV(int H, int S, int V) {
        // loop through the buffer and set each led to the desired color
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setHSV(i, H, S, V);
        }
    }

    /**
     * creates a command to set the entire strip to a rgb color
     * @return the generated command
     */
    public Command setColorRGBCommand(int r, int g, int b) {
        return this.runOnce(() -> setColorRGB(r, g, b));
    }

    /**
     * set the color of each led individually
     */
    public void setIndividualColors(int[] r, int[] g, int[] b) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, r[i], g[i], b[i]);
        }
    }

    /**
     * create a command to show a rgb color for a specified time
     * @param time the amount of time
     * @return the generated command
     */
    public Command showColorTime(int r, int g, int b, double time) {
        return runOnce(() -> setColorRGB(r, g, b)).andThen(Commands.waitSeconds(time));
    }

    private void updateAnimation() {
        if (!state.animated) {
            setColorRGB(state.r, state.g, state.b);
        }
        else {
            if (state.animationType.equals("flash")) {
                if (Timer.getFPGATimestamp() - lastAnimUpdate >= state.time) {
                    lastAnimUpdate = Timer.getFPGATimestamp();
                    lightsOn = !lightsOn;
                    if (lightsOn) {
                        setColorRGB(state.r, state.g, state.b);
                    } else {
                        setColorRGB(0, 0, 0);
                    }
                }
            }
            else if (state.animationType.equals("rainbow")) {
                lastAnimUpdate = Timer.getFPGATimestamp();
                rainbowH += 1;
                if (rainbowH >= 256) {
                    rainbowH = 0;
                }
                setColorHSV(rainbowH, rainbowS, rainbowV);
            }
            else if (state.animationType.equals("timed color")) {
                setColorRGB(state.r, state.g, state.b);
                if (Timer.getFPGATimestamp() - lastAnimUpdate >= state.time) {
                    lastAnimUpdate = Timer.getFPGATimestamp();
                    setState(Constants.LEDStates.nothing);
                }
            }
        }
    }

    public void setState(Constants.LEDStates state) {
        this.state = state;
    }
}

