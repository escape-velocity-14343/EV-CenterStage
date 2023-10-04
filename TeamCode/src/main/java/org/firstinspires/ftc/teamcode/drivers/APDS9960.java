package org.firstinspires.ftc.teamcode.drivers;

import static org.firstinspires.ftc.teamcode.drivers.APDS9960.SensingMode.COLOR;
import static org.firstinspires.ftc.teamcode.drivers.APDS9960.SensingMode.GESTURE;
import static org.firstinspires.ftc.teamcode.drivers.APDS9960.SensingMode.NONE;
import static org.firstinspires.ftc.teamcode.drivers.APDS9960.SensingMode.OFF;
import static org.firstinspires.ftc.teamcode.drivers.APDS9960.SensingMode.PROXIMITY;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

@I2cDeviceType
@DeviceProperties(name = "APDS9960 Color/Proximity/Gesture Sensor", xmlTag = "APDS9960")
public class APDS9960 extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    public final static I2cAddr ADDRESS = new I2cAddr(0x39);

    SensingMode currentMode = NONE;
    public APDS9960(I2cDeviceSynch i2cDeviceSynch, boolean deviceClientIsOwned) {
        super(i2cDeviceSynch, deviceClientIsOwned);
        this.deviceClient.setI2cAddress(ADDRESS);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    enum SensingMode {
        PROXIMITY,
        COLOR,
        GESTURE,
        OFF,
        NONE
    }


    @Override
    protected boolean doInitialize() {
        setMode(NONE);

        int power = deviceClient.read8(0x80);


        return power!=0;
    }

    /**
     * Select the mode that the APDS9960 should use.  This should be called
     * upon startup to ensure that the device is returning what you expect it to return.
     * @param mode the mode that the sensor should use
     */
    public void setMode(SensingMode mode) {
        currentMode = mode;
        if (mode == PROXIMITY) {
            deviceClient.write8(0x80, 0b0100101);
            deviceClient.write8(0xAB, 0b0000000);
            deviceClient.write8(0x89, 0x20);
            deviceClient.write8(0x8B, 0xF0);
        }
        else if (mode == COLOR) {
            deviceClient.write8(0x80, 0b0000011);
            deviceClient.write8(0xAB, 0b0000000);
        }
        else if (mode == GESTURE) {
            deviceClient.write8(0x80, 0b1000001);
            deviceClient.write8(0xAB, 0b0000001);
        }
        else if (mode == OFF) {
            deviceClient.write8(0x80, 0b0000000);
            deviceClient.write8(0xAB, 0b0000000);
        }
        else if (mode == NONE) {
            deviceClient.write8(0x80, 0b00000001);
            deviceClient.write8(0xAB, 0b0000000);
        }
    }
    /**
     * Uses the PROXIMITY mode to give a rough distance of what is in front of the sensor.
     * @return The distance away from the APDS9960, with 0 being touching and 255 being around 2 inches
     */
    public int getProximity() {
        if (currentMode != SensingMode.PROXIMITY) {
            Log.println(Log.WARN, "apds9960", "Sensor is in wrong mode, switching to PROXIMITY mode");
            setMode(PROXIMITY);
        }

        return 255-Byte.toUnsignedInt(deviceClient.read8(0x9C));
    }
    /**
     * Gets the 3 RGB colors from the APDS9960, and combines them into a <code>NormalizedRGBA</code>
     * @return The <code>NormalizedRGBA</code> with all 3 colors, but no alpha channel to save I2C calls
     */
    public NormalizedRGBA getColor() {
        if (currentMode != COLOR) {
            Log.println(Log.WARN, "apds9960", "Sensor is in wrong mode, switching to COLOR mode");
            setMode(COLOR);
        }
        NormalizedRGBA color = new NormalizedRGBA();
        //color.alpha = deviceClient.read8(0x94) + 256*deviceClient.read8(0x95);
        color.red = getRed();
        color.green = getGreen();
        color.blue = getBlue();
        return color;
    }
    public float getRed() {
        if (currentMode != COLOR) {
            Log.println(Log.WARN, "apds9960", "Sensor is in wrong mode, switching to COLOR mode");
            setMode(COLOR);
        }
        return (Byte.toUnsignedInt(deviceClient.read8(0x96)) + 256 * Byte.toUnsignedInt(deviceClient.read8(0x97)))/256f;
    }
    public float getGreen() {
        if (currentMode != COLOR) {
            Log.println(Log.WARN, "apds9960", "Sensor is in wrong mode, switching to COLOR mode");
            setMode(COLOR);
        }
        return (Byte.toUnsignedInt(deviceClient.read8(0x98)) + 256 * Byte.toUnsignedInt(deviceClient.read8(0x99)))/256f;
    }
    public float getBlue() {
        if (currentMode != COLOR) {
            Log.println(Log.WARN, "apds9960", "Sensor is in wrong mode, switching to COLOR mode");
            setMode(COLOR);
        }
        return (Byte.toUnsignedInt(deviceClient.read8(0x9A)) + 256 * Byte.toUnsignedInt(deviceClient.read8(0x9B)))/256f;
    }
    public byte getUp() {
        if (currentMode != GESTURE) {
            Log.println(Log.WARN, "apds9960", "Sensor is in wrong mode, switching to GESTURE mode");
            setMode(GESTURE);
        }
        return deviceClient.read8(0xFC);
    }
    public void setATime(int time) {
        deviceClient.write8(0x81, time);
    }
    public void forceInt(int val) {
        deviceClient.write8(0xE4, val);
    }
    public byte pint() {
        return deviceClient.read8(0x93);
    }


    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "APDS9960 Color/Proximity/Gesture Sensor";
    }

    @Override
    public void close() {
        setMode(OFF);
    }

    /**
     *
     * @param hmap the current robot hardware map
     * @param name the name set in the configuration
     * @return an initialized APDS9960
     */
    public static APDS9960 fromHMap(HardwareMap hmap, String name) {
        return hmap.get(APDS9960.class, name);
    }
}
