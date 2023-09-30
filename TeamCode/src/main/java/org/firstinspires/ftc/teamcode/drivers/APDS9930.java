package org.firstinspires.ftc.teamcode.drivers;

import android.util.Log;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;

@Deprecated
//@I2cDeviceType
@DeviceProperties(name = "APDS9930 Color/Proximity/Gesture Sensor", xmlTag = "APDS9930")
public class APDS9930 extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    public final static I2cAddr ADDRESS = new I2cAddr(0x39);
    public APDS9930(I2cDeviceSynch i2cDeviceSynch, boolean deviceClientIsOwned) {
        super(i2cDeviceSynch, deviceClientIsOwned);
        this.deviceClient.setI2cAddress(ADDRESS);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }
    @Deprecated
    @Override
    protected boolean doInitialize() {
        /*deviceClient.write8(0x80, 0b0000101);

        int power = deviceClient.read8(0x80);


        return power!=0;*/


        deviceClient.write8(0x00, 0); // turn off device

        // device settings: wait, als wait, prox wait, prox pulse
        deviceClient.write8(0x01, 0xff);
        deviceClient.write8(0x02, 0xff);
        deviceClient.write8(0x03, 0xff);
        deviceClient.write8(0x0e, 1);

        // prox/ch0 prox persistence
        deviceClient.write8(0x0c, 0b11111111);

        // proximity interrupt thresholds
        deviceClient.write8(0x08, 0b00000000);
        deviceClient.write8(0x09, 0b00000000);
        deviceClient.write8(0x0a, 0b11111111);
        deviceClient.write8(0x0b, 0b11111111);

        deviceClient.write8(0x0f, 0x20);

        deviceClient.write8(0x00, 0b00000101);

        String status = Integer.toBinaryString(deviceClient.read8(0x13));
        Log.println(Log.INFO,"apds9960",status);
        return true;
        //return status.charAt(6)==1;
    }
    public int getProximity() {
        //deviceClient.write8(0x80, 0b10100000);
        String status = Integer.toBinaryString(deviceClient.read8(0x13));
        Log.println(Log.INFO,"apds9960",status);
        return deviceClient.read8(0x18)+deviceClient.read8(0x19)*256;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "APDS9930 Color/Proximity/Gesture Sensor";
    }
}
