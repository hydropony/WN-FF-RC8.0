package org.firstinspires.ftc.teamcode.misc;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

/*
 * Created by Dryw Wade
 *
 * Driver for Adafruit's MCP9808 temperature sensor
 *
 * This version of the driver does not make use of the I2C device with parameters. This means the
 * settings for the configuration register are hard coded and cannot be changed by the user, nor can
 * they be different for each OpMode.
 */
@SuppressWarnings({"WeakerAccess", "unused"}) // Ignore access and unused warnings
// Both driver classes cannot register the sensor at the same time. One driver should have the
// sensor registered, and the other should be commented out
@I2cDeviceType
@DeviceProperties(name = "Amperka Proximity Sensor", description = "an Amperka Proximity Sensor", xmlTag = "APSENSOR")
public class AmperkaProximitySensor extends I2cDeviceSynchDevice<I2cDeviceSynch>
{
    ElapsedTime t;
    Telemetry tele = null;
    ////////////////////////////////////////////////////////////////////////////////////////////////
    // User Methods
    ////////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Raw Register Reads
    //////////////////////////////////////////////////////////////////////////////////////////////// RESULT__RANGE_VAL
    public int getDistance() throws InterruptedException {
        writeByte(Register.SYSRANGE__START, (byte) 0x01); // Start Single shot mode
//        wait(10);
        t = new ElapsedTime();
        while (t.milliseconds() < 10) {}
        writeByte(Register.SYSTEM__INTERRUPT_CLEAR, (byte) 0x07);
        return readByte(Register.RESULT__RANGE_VAL);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Read and Write Methods
    ////////////////////////////////////////////////////////////////////////////////////////////////
    public void addTelemetry(Telemetry telemetry) {
        this.tele = telemetry;
    }

    protected void writeByte(Register reg, byte value) {
//        byte[] mas = new byte[1];
//        mas[0] = value;
        deviceClient.write8(reg.bVal, value);
//        if (tele != null) {
//            tele.addData("Mas 0", mas[0]);
//            tele.addData("mas size", mas.length);
//            tele.update();
//        }
    }

    protected void writeByte(short reg, byte value) {
//        byte[] mas = new byte[1];
//        mas[0] = value;
        deviceClient.write8(reg, value);
//        if (tele != null) {
//            tele.addData("Mas 0", mas[0]);
//            tele.addData("mas size", mas.length);
//            tele.update();
//        }
    }

    protected void writeShort(final Register reg, short value)
    {
        deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value));
    }

    protected void writeShort(short reg, short value)
    {
        deviceClient.write(reg, TypeConversion.shortToByteArray(value));
    }

    protected short readShort(Register reg)
    {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
    }

    protected int readByte(Register reg)
    {
        return (int)deviceClient.read8(reg.bVal);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Registers and Config Settings
    ////////////////////////////////////////////////////////////////////////////////////////////////

    public enum Register
    {
        FIRST(0x000),
        IDENTIFICATION__MODEL_ID(0x000),
        IDENTIFICATION__MODEL_REV_MAJOR(0x001),
        IDENTIFICATION__MODEL_REV_MINOR(0x002),
        IDENTIFICATION__MODULE_REV_MAJOR(0x003),
        IDENTIFICATION__MODULE_REV_MINOR(0x004),
        IDENTIFICATION__DATE_HI(0x006),
        IDENTIFICATION__DATE_LO(0x007),
        IDENTIFICATION__TIME1(0x008),
        IDENTIFICATION__TIME2(0x009),
        SYSTEM__MODE_GPIO0(0x010),
        SYSTEM__MODE_GPIO1(0x011),
        SYSTEM__HISTORY_CTRL(0x012),
        SYSTEM__INTERRUPT_CONFIG_GPIO(0x014),
        SYSTEM__INTERRUPT_CLEAR(0x015),
        SYSTEM__FRESH_OUT_OF_RESET(0x016),
        SYSTEM__GROUPED_PARAMETER_HOLD(0x017),
        SYSRANGE__START(0x018),
        SYSRANGE__THRESH_HIGH(0x019),
        SYSRANGE__THRESH_LOW(0x01A),
        SYSRANGE__INTERMEASUREMENT_PERIOD(0x01B),
        SYSRANGE__MAX_CONVERGENCE_TIME(0x01C),
        SYSRANGE__CROSSTALK_COMPENSATION_RATE(0x01E),
        SYSRANGE__CROSSTALK_VALID_HEIGHT(0x021),
        SYSRANGE__EARLY_CONVERGENCE_ESTIMATE(0x022),
        SYSRANGE__PART_TO_PART_RANGE_OFFSET(0x024),
        SYSRANGE__RANGE_IGNORE_VALID_HEIGHT(0x025),
        SYSRANGE__RANGE_IGNORE_THRESHOLD(0x026),
        SYSRANGE__MAX_AMBIENT_LEVEL_MULT(0x02C),
        SYSRANGE__RANGE_CHECK_ENABLES(0x02D),
        SYSRANGE__VHV_RECALIBRATE(0x02E),
        SYSRANGE__VHV_REPEAT_RATE(0x031),
        SYSALS__START(0x038),
        SYSALS__THRESH_HIGH(0x03A),
        SYSALS__THRESH_LOW(0x03C),
        SYSALS__INTERMEASUREMENT_PERIOD(0x03E),
        SYSALS__ANALOGUE_GAIN(0x03F),
        SYSALS__INTEGRATION_PERIOD(0x040),
        RESULT__RANGE_STATUS(0x04D),
        RESULT__ALS_STATUS(0x04E),
        RESULT__INTERRUPT_STATUS_GPIO(0x04F),
        RESULT__ALS_VAL(0x050),
        RESULT__HISTORY_BUFFER_x(0x052), //??
        RESULT__RANGE_VAL(0x062),
        RESULT__RANGE_RAW(0x064),
        RESULT__RANGE_RETURN_RATE(0x066),
        RESULT__RANGE_REFERENCE_RATE(0x068),
        RESULT__RANGE_RETURN_SIGNAL_COUNT(0x06C),
        RESULT__RANGE_REFERENCE_SIGNAL_COUNT(0x070),
        RESULT__RANGE_RETURN_AMB_COUNT(0x074),
        RESULT__RANGE_REFERENCE_AMB_COUNT(0x078),
        RESULT__RANGE_RETURN_CONV_TIME(0x07C),
        RESULT__RANGE_REFERENCE_CONV_TIME(0x080),
        READOUT__AVERAGING_SAMPLE_PERIOD(0x10A), //2byte
//        FIRMWARE__BOOTUP(0x119),
        FIRMWARE__RESULT_SCALER(0x120), //2byte
//        I2C_SLAVE__DEVICE_ADDRESS(0x212),
//        INTERLEAVED_MODE__ENABLE(0x2A3),
        LAST(0x2A3);


        public short bVal;

        Register(int bVal)
        {
            this.bVal = (short) bVal;
        }
    }

    // More settings are available on the sensor, but not included here. Could be added later

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Construction and Initialization
    ////////////////////////////////////////////////////////////////////////////////////////////////

    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x29);

    public AmperkaProximitySensor(I2cDeviceSynch deviceClient) throws RuntimeException
    {
        super(deviceClient, true);

//        this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        super.registerArmingStateCallback(false); // Deals with USB cables getting unplugged
        // Sensor starts off disengaged so we can change things like I2C address. Need to engage
        this.deviceClient.engage();
    }

    protected void setOptimalReadWindow()
    {
        // Sensor registers are read repeatedly and stored in a register. This method specifies the
        // registers and repeat read mode
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                Register.FIRST.bVal,
                Register.LAST.bVal - Register.FIRST.bVal + 1,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }

    public void setDefaultSettings() {
        writeByte(Register.SYSTEM__INTERRUPT_CONFIG_GPIO, (byte) ((4 << 3) | (4))); // Set GPIO1 high when sample complete

        writeByte(Register.SYSTEM__MODE_GPIO1, (byte) 0x10);               // Set GPIO1 high when sample complete
        writeByte(Register.READOUT__AVERAGING_SAMPLE_PERIOD, (byte) 0x30); // Set Avg sample period
        writeByte(Register.SYSALS__ANALOGUE_GAIN, (byte) 0x46);            // Set the ALS gain
        writeByte(Register.SYSRANGE__VHV_REPEAT_RATE, (byte) 0xFF);        // Set auto calibration period (Max = 255)/(OFF = 0)
        writeByte(Register.SYSALS__INTEGRATION_PERIOD, (byte) 0x63);       // Set ALS integration time to 100ms
        writeByte(Register.SYSRANGE__VHV_RECALIBRATE, (byte) 0x01);        // perform a single temperature calibration
        // Optional settings from datasheet
        // http://www.st.com/st-web-ui/static/active/en/resource/technical/document/application_note/DM00122600.pdf
        writeByte(Register.SYSRANGE__INTERMEASUREMENT_PERIOD, (byte) 0x09); // Set default ranging inter-measurement period to 100ms
        writeByte(Register.SYSALS__INTERMEASUREMENT_PERIOD, (byte) 0x0A);   // Set default ALS inter-measurement period to 100ms
        writeByte(Register.SYSTEM__INTERRUPT_CONFIG_GPIO, (byte) 0x24);     // Configures interrupt on ‘New Sample Ready threshold event’
        // Additional settings defaults from community
        writeByte(Register.SYSRANGE__MAX_CONVERGENCE_TIME, (byte) 0x32);
        writeByte(Register.SYSRANGE__RANGE_CHECK_ENABLES, (byte) (0x10 | 0x01));
        writeShort(Register.SYSRANGE__EARLY_CONVERGENCE_ESTIMATE, (byte) 0x7B);
        writeShort(Register.SYSALS__INTEGRATION_PERIOD, (byte) 0x64);

        writeByte(Register.READOUT__AVERAGING_SAMPLE_PERIOD, (byte) 0x30);
        writeByte(Register.SYSALS__ANALOGUE_GAIN, (byte) 0x40);
        writeByte(Register.FIRMWARE__RESULT_SCALER, (byte) 0x01);
    }

    @Override
    protected synchronized boolean doInitialize()
    {
        return true;
    }

    public boolean init() {
        int data; // for temp data storage

        data = readByte(Register.SYSTEM__FRESH_OUT_OF_RESET);

        if (data != 1)
            return true;

        writeByte((short) 0x0207, (byte) 0x01);
        writeByte((short) 0x0208, (byte)0x01);
        writeByte((short) 0x0096, (byte)0x00);
        writeByte((short) 0x0097, (byte)0xfd);
        writeByte((short) 0x00e3, (byte)0x00);
        writeByte((short) 0x00e4, (byte)0x04);
        writeByte((short) 0x00e5, (byte)0x02);
        writeByte((short) 0x00e6, (byte)0x01);
        writeByte((short) 0x00e7, (byte)0x03);
        writeByte((short) 0x00f5, (byte)0x02);
        writeByte((short) 0x00d9, (byte)0x05);
        writeByte((short) 0x00db, (byte)0xce);
        writeByte((short) 0x00dc, (byte)0x03);
        writeByte((short) 0x00dd, (byte)0xf8);
        writeByte((short) 0x009f, (byte)0x00);
        writeByte((short) 0x00a3, (byte)0x3c);
        writeByte((short) 0x00b7, (byte)0x00);
        writeByte((short) 0x00bb, (byte)0x3c);
        writeByte((short) 0x00b2, (byte)0x09);
        writeByte((short) 0x00ca, (byte)0x09);
        writeByte((short) 0x0198, (byte)0x01);
        writeByte((short) 0x01b0, (byte)0x17);
        writeByte((short) 0x01ad, (byte)0x00);
        writeByte((short) 0x00ff, (byte)0x05);
        writeByte((short) 0x0100, (byte)0x05);
        writeByte((short) 0x0199, (byte)0x05);
        writeByte((short) 0x01a6, (byte)0x1b);
        writeByte((short) 0x01ac, (byte)0x3e);
        writeByte((short) 0x01a7, (byte)0x1f);
        writeByte((short) 0x0030, (byte)0x00);
        setDefaultSettings();
        if (tele != null) {
            tele.addData("Init is successfull", null);
            tele.update();
        }
        return false;
    }

    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName()
    {
        return "nnsdsd";
    }
}