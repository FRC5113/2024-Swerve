#pragma once
#include <algorithm> 
#include <chrono>
#include <cstdint>
#include <frc/I2C.h>
#include <frc/util/Color.h>


namespace frc {

class TMD37003 {
  public:
    /**
     * Create Instance of TMD3700 color sensor driver.
     * 
     * @param i2cPort Internal/MXP I2C port on the roboRIO
     */
    TMD37003(I2C::Port i2cPort);


    /**
     * Configure TMD3700 Color (Ambient Light Sensing) parameters.
     * 
     * @param alsIntegrationTime Color sensing sample time in milliseconds.  Value may
     *                           range from 2.8 to 721ms.   Longer sample times act to
     *                           filtered the sampled color.
     * @param alsGain Color sensor gain as a value between 1 and 64.
     */
    void ConfigureColorSense(double alsIntegrationTime, int alsGain);


    /**
     * Configure TMD3700 Proximity sense parameters.
     * 
     * @param proximitySampleTime  Proximity sensing sample time in milliseconds.  Value
     *                             may range from 0.088 to 22.528 ms.
     * @param proximityPulseLength Lengh of each IR LED pulse during proximity measurement
     *                             in milliseconds.  Value must fall between 0.004 and 0.032 ms.
     * @param numProximityPulses   Number of proximity IR LED pulses which occur during each
     *                             sample period
     * @param proximityGain        Proximity sensor gain as a value between 1 and 8.
     * @param proximityLedCurrent  Proximity IR LED current in milliamps.  Value must fall
     *                             between 6 and 192 mA
     */
    void ConfigureProximitySense(double proximitySampleTime, double proximityPulseLength, int numProximityPulses, 
                                 int proximityGain, int proximityLedCurrent);


    /**
     * Specifiy gains and gamma value to convert raw RGB samples to normalized
     * RGB values to aproximate sRGB space.
     * 
     * <p>The default gains are calibrated for the built in white LED.  If another
     * lighting source is used this function may be required to specify the
     * white point.
     * 
     * <p> Channels are calculated using:
     * <p>{Normilized value} = ({Raw value} * gain) ^ (1/gamma)
     * 
     * @param r Red channel gain
     * @param g Green channel gain
     * @param b Blue channel gain
     * @param c Clear (ambient) channel gain
     * @param gamma Gamma vaulke used to convert raw (linear) samples to something
     *              that responds like a human eye
     */
    void SetGain(double r, double g, double b, double c, double gamma);


    /**
     * Get gamma corrected RGB values from sensor
     * 
     * @return Value of RGB samples
     */ 
    Color GetColor();


    /**
     * Get the measured color (hue).
     * 
     * @return Measured hue in degrees
     */
    double GetHue();


    /**
     * Get measured color saturation.
     * 
     * @return Measured saturation as ratio between 0 and 1
     */
    double GetSaturation();


    /** Get red channel value.
     * 
     * @return Normalized red channel value as ratio between 0 and 1..
     */
    double GetRed();


    /** Get green channel value.
     * 
     * @return Normalized green channel value as ratio between 0 and 1..
     */
    double GetGreen();


    /** Get blue channel value.
     * 
     * @return Normalized blue channel value as ratio between 0 and 1..
     */
    double GetBlue();

    /** Get clear (Ambient) channel value.
     * 
     * @return Normalized clear channel value as ratio between 0 and 1.
     */
    double GetAmbientLightLevel();


    /** Get proximity value.
     * 
     * @return Normalized proximity value as ratio between 0 and 1.
     */
    double GetProximity();
        

    private:
        const uint8_t  m_deviceAddress = 0x39;
        I2C m_port;
        std::chrono::steady_clock::time_point lastRx;

        bool m_alsSaturated;
        bool m_proximitySaturated;
        double m_r;
        double m_g;
        double m_b;
        double m_c;
        double m_proximity;
 
        double m_alsIntegrationTime     = 80;
        double m_alsWaitTime            =  0;
        int    m_alsGain                =  4;
        double m_proximitySampleTime    = 20;
        double m_proximityPulseLength   = 0.008;
        int    m_numProximityPulses     =  2;
        int    m_proximityGain          =  2;
        int    m_proximityLedCurrent    = 90;

        double m_gamma = 1.0 / 2.2;
        double m_rGain = 1.0 / 0.197;
        double m_gGain = 1.0 / 0.126;
        double m_bGain = 1.0 / 0.071;
        double m_cGain = 1.0 / 0.340;

        enum Reg {
            ENABLE    = 0x80,
            ATIME     = 0x81,
            PRATE     = 0x82,
            WTIME     = 0x83,
            AILTL     = 0x84,
            AILTH     = 0x85,
            AIHTL     = 0x86,
            AIHTH     = 0x87,
            PILT      = 0x88,
            PIHT      = 0x8A,
            PERS      = 0x8C,
            CFG0      = 0x8D,
            PCFG0     = 0x8E,
            PCFG1     = 0x8F,
            CFG1      = 0x90,
            REVID     = 0x91,
            ID        = 0x92,
            STATUS    = 0x93,
            CDATAL    = 0x94,
            CDATAH    = 0x95,
            RDATAL    = 0x96,
            RDATAH    = 0x97,
            GDATAL    = 0x98,
            GDATAH    = 0x99,
            BDATAL    = 0x9A,
            BDATAH    = 0x9B,
            PDATA     = 0x9C,
            CFG2      = 0x9F,
            CFG3      = 0xAB,
            POFFSET_L = 0xC0,
            POFFSET_H = 0xC1,
            CALIB     = 0xD7,
            CALIBCFG  = 0xD9,
            CALIBSTAT = 0xDC,
            INTENAB   = 0xDD
        };
    
    void PushConfiguration();

     void Read();
};

}