// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LightStrip {

  /**
   * Creates a new Lights.
   */
  public enum ledColors {

    RED(redColor),
    PURPLE(purpleColor),
    YELLOW(yellowColor),
    GREEN(greenColor),
    BLUE(blueColor);

    int[] rgb = new int[3];

    private ledColors(int[] rgb) {
      this.rgb = rgb;
    }

    public int[] getColor() {
      return rgb;
    }
  }

  public int m_numOfLEDs;

  private AddressableLED m_led;

  private AddressableLEDBuffer m_ledBuffer;
  // Store what the last hue of the first pixel is

  private ledColors currentColor;

  static int[] rgb = { 0, 0, 0 };

  private static int[] purpleColor = { 105, 17, 122 };

  private static int[] yellowColor = { 122, 112, 17 };

  private static int[] redColor = { 255, 0, 0 };

  private static int[] blueColor = { 0, 0, 255 };

  private static int[] greenColor = { 0, 255, 0 };

  private static boolean colorChange = false;

  public LightStrip(int port, int numOfLEDs) {

    m_led = new AddressableLED(port);
    m_numOfLEDs = numOfLEDs;
    m_ledBuffer = new AddressableLEDBuffer(numOfLEDs);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();

    // Set up thread properties and start it off
    monitorThread.setName("LEDThread");
    monitorThread.setPriority(Thread.MIN_PRIORITY);
    monitorThread.start();
  }

  public void setColor(ledColors color) {

    rgb = color.getColor();

    colorChange = true;

    currentColor = color;

  }

  public void togglePY() {
    if (currentColor == ledColors.PURPLE)
      setColor(ledColors.YELLOW);
    else
      setColor(ledColors.PURPLE);

  }

  Thread monitorThread = new Thread(new Runnable() {
    @Override
    public void run() {
      try {
        while (!Thread.currentThread().isInterrupted()) {

          if (colorChange) {

            for (int i = 0; i < m_numOfLEDs; i++) {

              m_ledBuffer.setRGB(i, rgb[0], rgb[1], rgb[2]);

            }
            m_led.setData(m_ledBuffer);

            colorChange = false;

          }
          Thread.sleep(100);
        }
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
  });

}