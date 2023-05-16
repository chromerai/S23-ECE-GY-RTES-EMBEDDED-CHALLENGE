#include <mbed.h>
#include <stdio.h>
#include "drivers/LCD_DISCO_F429ZI.h"

// Example 2
InterruptIn int2(PA_2, PullDown);
SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel); // mosi, miso, sclk, cs
// address of first register with gyro data
#define OUT_X_L 0x28
// register fields(bits): data_rate(2),Bandwidth(2),Power_down(1),Zen(1),Yen(1),Xen(1)
#define CTRL_REG1 0x20
// configuration: 200Hz ODR,50Hz cutoff, Power on, Z on, Y on, X on
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1
// register fields(bits): reserved(1), endian-ness(1),Full scale sel(2), reserved(1),self-test(2), SPI mode(1)
#define CTRL_REG4 0x23
// configuration: reserved,little endian,500 dps,reserved,disabled,4-wire mode
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0
// register fields(bits): I1_Int1 (1), I1_Boot(1), H_Lactive(1), PP_OD(1), I2_DRDY(1), I2_WTM(1), I2_ORun(1), I2_Empty(1)
#define CTRL_REG3 0x22
// configuration: Int1 disabled, Boot status disabled, active high interrupts, push-pull, enable Int2 data ready, disable fifo interrupts
#define CTRL_REG3_CONFIG 0b0'0'0'0'1'000

#define SPI_FLAG 1
#define DATA_READY_FLAG 2
#define SCALING_FACTOR (17.5f * 0.017453292519943295769236907684886f / 1000.0f)
#define LINE(x) ((x) * (((sFONT *)BSP_LCD_GetFont())->Height))

// buffer for reading and writing data to the gyro
uint8_t write_buf[32];
uint8_t read_buf[32];

// initialize the LCD
LCD_DISCO_F429ZI lcd;

EventFlags flags;
// the spi.transfer() function requires that the callback
// provided to it takes an int parameter
void spi_cb(int event)
{
    flags.set(SPI_FLAG);
}

// The interrupt handler for the data ready interrupt
void data_cb()
{
    flags.set(DATA_READY_FLAG);
};

DigitalIn button(BUTTON1);
DigitalOut led(LED_RED);
DigitalOut led1(LED1);
bool recording_started = true;
Timer timer;

struct MotionSample
{
    float gx;
    float gy;
    float gz;
};

// low pass filter
void applyLowPassFilter(MotionSample *motionData, float alpha)
{
    // initialize the filtered motion data array
    MotionSample filteredMotionData[50];
    filteredMotionData[0] = motionData[0];

    // apply the low-pass filter to the motion data
    for (int i = 1; i < 50; i++)
    {
        filteredMotionData[i].gx = alpha * motionData[i].gx + (1 - alpha) * filteredMotionData[i - 1].gx;
        filteredMotionData[i].gy = alpha * motionData[i].gy + (1 - alpha) * filteredMotionData[i - 1].gy;
        filteredMotionData[i].gz = alpha * motionData[i].gz + (1 - alpha) * filteredMotionData[i - 1].gz;
    }

    // copy the filtered motion data back into the original array
    for (int i = 0; i < 50; i++)
    {
        motionData[i] = filteredMotionData[i];
    }
}

float calculateEuclideanDistance(struct MotionSample recorded, struct MotionSample unlock)
{
    // calculate the difference between the recorded and unlock motion data
    float diff_gx = (recorded.gx - unlock.gx);
    float diff_gy = (recorded.gy - unlock.gy);
    float diff_gz = (recorded.gz - unlock.gz);

    // return the euclidean distance between the two motion data sets
    return sqrt(diff_gx * diff_gx + diff_gy * diff_gy + diff_gz * diff_gz);
}

int main()
{
    // setup the spi for 8 bit data, high steady state clock,
    // second edge capture, with a 1MHz clock rate

    spi.format(8, 3);
    spi.frequency(1'000'000);

    write_buf[0] = CTRL_REG1;
    write_buf[1] = CTRL_REG1_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb, SPI_EVENT_COMPLETE);
    flags.wait_all(SPI_FLAG);

    write_buf[0] = CTRL_REG4;
    write_buf[1] = CTRL_REG4_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb, SPI_EVENT_COMPLETE);
    flags.wait_all(SPI_FLAG);

    int whoami = read_buf[1];
    printf("WHOAMI register = 0x%X\n", whoami);

    // Configure the interrupt to call our function
    // When the pin becomes high
    int2.rise(&data_cb);

    write_buf[0] = CTRL_REG3;
    write_buf[1] = CTRL_REG3_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb, SPI_EVENT_COMPLETE);
    flags.wait_all(SPI_FLAG);

    // This is where to store the data, a struct with 1000 elements
    // array of motionsamples, can hold up to 100 elements
    MotionSample recordedData[50];
    MotionSample unlockData[50];
    // In the while loop, when button is pressed the first time, there will be 2 seconds interval to record
    // the motion data, then it will stop, when the button is pressed again, it will start recording the data in to
    // a new array and after 5 seconds, it will try to compare the two arrays of structs, to see if they match, if they do
    // then unlocked, else its not

    while (1)
    {
        // start recording the motion when the button is pressed
        lcd.DisplayStringAt(0, LINE(2), (uint8_t *)"Ready to record", CENTER_MODE);
        lcd.DisplayStringAt(0, LINE(4), (uint8_t *)"Press button to start", CENTER_MODE);
        lcd.DisplayStringAt(0, LINE(6), (uint8_t *)"Recording time: 4s", CENTER_MODE);
        if (button.read() == 1 && recording_started == true)
        {
            led.write(0);
            led1.write(0);
            lcd.Clear(LCD_COLOR_WHITE);
            recording_started = true;
            lcd.DisplayStringAt(0, LINE(2), (uint8_t *)"Recording in progress", CENTER_MODE);
            printf("start recording, creating new motion \n");
            timer.start();
            int i = 0;
            while (timer.read_ms() < 4000)
            {
                int16_t raw_gx, raw_gy, raw_gz;
                float gx, gy, gz;
                // prepare the write buffer to trigger a sequential read
                write_buf[0] = OUT_X_L | 0x80 | 0x40;
                // start sequential sample reading
                spi.transfer(write_buf, 7, read_buf, 7, spi_cb, SPI_EVENT_COMPLETE);
                flags.wait_all(SPI_FLAG);
                // read_buf after transfer: garbage byte, gx_low,gx_high,gy_low,gy_high,gz_low,gz_high
                // Put the high and low bytes in the correct order lowB,HighB -> HighB,LowB
                raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t)read_buf[1]);
                raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t)read_buf[3]);
                raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t)read_buf[5]);

                printf("RAW|\tgx: %d \t gy: %d \t gz: %d\t\n", raw_gx, raw_gy, raw_gz);

                gx = ((float)raw_gx) * SCALING_FACTOR;
                gy = ((float)raw_gy) * SCALING_FACTOR;
                gz = ((float)raw_gz) * SCALING_FACTOR;

                recordedData[i].gx = gx;
                recordedData[i].gy = gy;
                recordedData[i].gz = gz;

                printf("Actual|\tgx: %4.5f \t gy: %4.5f \t gz: %4.5f \n", recordedData[i].gx, recordedData[i].gy, recordedData[i].gz);
                i++;
                led.write(1);
                thread_sleep_for(100);
                led.write(0);
            }
            timer.stop();  // stop the countdown
            timer.reset(); // reset the countdown to 0

            lcd.Clear(LCD_COLOR_WHITE);
            lcd.DisplayStringAt(0, LINE(3), (uint8_t *)"Motion recorded", CENTER_MODE);
            recording_started = false;

            lcd.DisplayStringAt(0, LINE(6), (uint8_t *)"Locked!", CENTER_MODE);
            // exit recording loop and go into unlock loop
            break;
        }
    }
    thread_sleep_for(1000);
    printf("Unlock the device?\n");
    while (1)
    {
        if (button == 1 && recording_started == false)
        {
            printf("start unlocking \n");
            lcd.Clear(LCD_COLOR_WHITE);
            lcd.DisplayStringAt(0, LINE(3), (uint8_t *)"Motion recorded", CENTER_MODE);
            lcd.DisplayStringAt(0, LINE(6), (uint8_t *)"Attempting unlock", CENTER_MODE);
            timer.start();
            int i = 0;
            float totalDistance = 0.0; // total distance between the two motions
            while (timer.read_ms() < 2000)
            {
                int16_t raw_gx, raw_gy, raw_gz;
                float gx, gy, gz;
                // prepare the write buffer to trigger a sequential read
                write_buf[0] = OUT_X_L | 0x80 | 0x40;
                // start sequential sample reading
                spi.transfer(write_buf, 7, read_buf, 7, spi_cb, SPI_EVENT_COMPLETE);
                flags.wait_all(SPI_FLAG);
                // read_buf after transfer: garbage byte, gx_low,gx_high,gy_low,gy_high,gz_low,gz_high
                // Put the high and low bytes in the correct order lowB,HighB -> HighB,LowB
                raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t)read_buf[1]);
                raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t)read_buf[3]);
                raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t)read_buf[5]);

                printf("RAW|\tgx: %d \t gy: %d \t gz: %d\t\n", raw_gx, raw_gy, raw_gz);

                gx = ((float)raw_gx) * SCALING_FACTOR;
                gy = ((float)raw_gy) * SCALING_FACTOR;
                gz = ((float)raw_gz) * SCALING_FACTOR;

                unlockData[i].gx = gx;
                unlockData[i].gy = gy;
                unlockData[i].gz = gz;

                printf("Actual|\tgx: %4.5f \t gy: %4.5f \t gz: %4.5f\t\n", unlockData[i].gx, unlockData[i].gy, unlockData[i].gz);
                i++;
                led1.write(1);
                thread_sleep_for(100);
                led1.write(0);
            }
            timer.stop();  // stop the countdown
            timer.reset(); // reset the countdown to 0

            // low pass filter
            applyLowPassFilter(unlockData, 0.95);
            applyLowPassFilter(recordedData, 0.95);

            // set threshold values for noise removal and for distance matching, tuned through trial and error
            // further adjustments can be made for better accuracy.
            float coordinateThreshold = 0.5, distanceThreshold = 0.5;

            // initialize match flag as true, assume the motion matches
            bool motionMatched = true;

            // counters for matched and mismatched data points.
            int matchCount = 0, mismatchCount = 0;

            // iterate over the data arrays.
            for (int i = 0; i < 50; i++)
            {
                // initialize flags for matching x, y, and z coordinates.
                bool xMatch = false, yMatch = false, zMatch = false;
                // initialize a flag for any match (coordinate or distance).
                bool anyMatch = false;

                // calculate the absolute difference between x-coordinates.
                float difference = fabs(unlockData[i].gx - recordedData[i].gx);

                // ignore data points where gx, gy, gz are just about nil
                if (difference < 0.000001)
                {
                    continue;
                }

                // if x-coordinates difference is within the threshold, set xMatch as true.
                if (difference < coordinateThreshold)
                {
                    xMatch = true;
                }

                // calculate the absolute difference between y-coordinates.
                difference = fabs(unlockData[i].gy - recordedData[i].gy);

                // if y-coordinates difference is within the threshold, set yMatch as true.
                if (difference < coordinateThreshold)
                {
                    yMatch = true;
                }

                // calculate the absolute difference between z-coordinates.
                difference = fabs(unlockData[i].gz - recordedData[i].gz);

                // if z-coordinates difference is within the threshold, set zMatch as true.
                if (difference < coordinateThreshold)
                {
                    zMatch = true;
                }

                // if any two of the three coordinates match, set anyMatch as true.
                if ((xMatch && yMatch) || (xMatch && zMatch) || (yMatch && zMatch))
                {
                    anyMatch = true;
                }

                // calculate the Euclidean distance between the two data points.
                float distance = calculateEuclideanDistance(recordedData[i], unlockData[i]);
                totalDistance += distance;

                // if Euclidean distance is less than the threshold, set anyMatch as true.
                if (distance < distanceThreshold)
                {
                    anyMatch = true;
                }

                // if any match (coordinate or distance) was found, increment the match counter.
                // and if no match was found, increment the mismatch counter.
                if (anyMatch)
                {
                    matchCount++;
                }
                else
                {
                    mismatchCount++;
                }
            }

            // print the counts of matching and mismatching data points.
            printf("%d, %d \n", matchCount, mismatchCount);

            // initialize and calculate the match score.
            float matchScore = 0;
            matchScore = (float)matchCount / (matchCount + mismatchCount);

            // print the match score and the total distance.
            printf("Match Score: %f\n", matchScore);
            printf("Total Distance: %f\n", totalDistance);

            // evaluate the match status based on total distance and match score.
            if (totalDistance == infinityf() || (totalDistance > 20 && matchScore > 0.9))
            {
                motionMatched = false;
            }
            else if (matchScore > 0.55)
            {
                printf("MATCHED\n");
                motionMatched = true;
            }
            else
            {
                printf("NOT MATCHED\n");
                motionMatched = false;
            }

            // clear LCD screen
            lcd.Clear(LCD_COLOR_WHITE);

            // display the recording status.
            lcd.DisplayStringAt(0, LINE(3), (uint8_t *)"Motion recorded", CENTER_MODE);

            // display unlock status and associated actions
            if (!motionMatched)
            {
                printf("Motion Unlocking Failed.\n");
                lcd.DisplayStringAt(0, LINE(8), (uint8_t *)"Unlock failed", CENTER_MODE);
                led.write(1);
                thread_sleep_for(1000);
            }
            else
            {
                printf("Motion Unlocked.\n");
                lcd.DisplayStringAt(0, LINE(8), (uint8_t *)"Device Unlocked", CENTER_MODE);
                led1.write(1);
                thread_sleep_for(1000);
            }

            // prompt the user for another unlock attempt.
            printf("Would you like to unlock?\n");
            lcd.DisplayStringAt(0, LINE(10), (uint8_t *)"Try again?", CENTER_MODE);
        }
    }
}