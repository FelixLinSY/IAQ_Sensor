#include <LWiFi.h>
#include <WiFiClient.h>
#include <SoftwareSerial.h>
#include "MCS.h"
#include <SimpleDHT.h>
#include "PMS.h"

PMS pms(Serial1);
PMS::DATA pm25_data;

SimpleDHT22 dht22;

#define FUNC_SENSOR_CO2     1
#define FUNC_SENSOR_PM25    1
#define FUNC_SENSOR_DHT22   1

#define PIN_CO2_RX          3
#define PIN_CO2_TX          4
#define PIN_DHT22           5
#define PIN_PM25_TX         6
#define PIN_PM25_RX         7

#define GET_DATA_INTERVAL   5000  /* in ms */

SoftwareSerial s_serial(PIN_CO2_RX, PIN_CO2_TX);   //RX, TX

// Assign AP ssid / password here
#define _SSID "WIFI_SSID"
#define _KEY  "WIFI_PASSWORD"

#define MCS_DEVICE_ID   "DEVICE_ID"
#define MCS_DEVICE_KEY  "DEVICE_KEY"

// Assign device id / key of your test device
MCSDevice mcs(MCS_DEVICE_ID, MCS_DEVICE_KEY);

// Assign channel id
MCSDisplayFloat     MCS_temp("temperature");
MCSDisplayFloat     MCS_humi("humidity");
MCSDisplayInteger   MCS_co2("co2");
MCSDisplayInteger   MCS_pm25("pm25");

const unsigned char mhz16_get_sensor[] = {
  0xff, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79
};

const unsigned char mhz16_calibrate[] = {
  0xff, 0x87, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf2
};

struct aqi {
  float temperature;
  float humidity;
  int co2;
  int pm1;
  int pm2_5;
  int pm10;
} my_aqi;


void setup() {
  // setup Serial debug output at 115200
  Serial.begin(115200);
  // setup Serial1 (hardware) for PMS3003 sensor, Tx:6 Rx:7
  Serial1.begin(9600);
  // setup software serial for MH-Z16 CO2 sensor
  s_serial.begin(9600);

  // setup Wifi connection
  while (WL_CONNECTED != WiFi.status()) {
    Serial.print("WiFi.begin(");
    Serial.print(_SSID);
    Serial.print(",");
    Serial.print(_KEY);
    Serial.println(")...");
    WiFi.begin(_SSID, _KEY);
  }
  Serial.println("WiFi connected !!");

  // setup MCS connection
  mcs.addChannel(MCS_temp);
  mcs.addChannel(MCS_humi);
  mcs.addChannel(MCS_co2);
  mcs.addChannel(MCS_pm25);
  while (!mcs.connected())
  {
    Serial.println("MCS.connect()...");
    mcs.connect();
  }
  Serial.println("MCS connected !!");

}

/* MH-Z16 calibrate function */
void calibrate() {
  Serial.println("begin to calibrate");

  for (int i = 0; i < sizeof(mhz16_calibrate); i++) {
    s_serial.write(mhz16_calibrate[i]);
  }

  Serial.println("calibrate done");
}

bool dataRecieve(void)
{
  int temperature;
  
  byte data[9];
  int i = 0;

  //transmit command data
  for (i = 0; i < sizeof(mhz16_get_sensor); i++) {
    s_serial.write(mhz16_get_sensor[i]);
  }
  delay(10);
  //begin reveiceing data
  if (s_serial.available()) {
    while (s_serial.available()) {
      for (int i = 0; i < 9; i++) {
        data[i] = s_serial.read();
      }
    }
  }

  for (int j = 0; j < 9; j++) {
    Serial.print(data[j]);
    Serial.print(" ");
  }
  Serial.println("");

  if ((i != 9) || (1 + (0xFF ^ (byte)(data[1] + data[2] + data[3] + data[4] + data[5] + data[6] + data[7]))) != data[8]) {
    return false;
  }

  my_aqi.co2 = (int)data[2] * 256 + (int)data[3];
  temperature = (int)data[4] - 40;

  return true;
}

void readDHT() {


  // read temperature & humidity from DHT22
  Serial.println("=================================");
  Serial.println("Sample DHT22...");

  int err = SimpleDHTErrSuccess;
  if ((err = dht22.read2(PIN_DHT22, &my_aqi.temperature, &my_aqi.humidity, NULL)) != SimpleDHTErrSuccess) {
    Serial.print("Read DHT22 failed, err=");
    Serial.println(err);
    //    delay(2000);
  } else {
    MCS_temp.set(my_aqi.temperature);
    MCS_humi.set(my_aqi.humidity);
  }

  Serial.print("Sample OK: ");
  Serial.print((float)my_aqi.temperature); Serial.print(" *C, ");
  Serial.print((float)my_aqi.humidity); Serial.println(" RH%");
}

int readPMS() {
  if (pms.read(pm25_data, 10000)) {
    my_aqi.pm1 = pm25_data.PM_AE_UG_1_0;
    my_aqi.pm2_5 = pm25_data.PM_AE_UG_2_5;
    my_aqi.pm10 = pm25_data.PM_AE_UG_10_0;

    /* print data */
    Serial.println("Data:");

    Serial.print("PM 1.0 (ug/m3): ");
    Serial.println(pm25_data.PM_AE_UG_1_0);

    Serial.print("PM 2.5 (ug/m3): ");
    Serial.println(pm25_data.PM_AE_UG_2_5);

    Serial.print("PM 10.0 (ug/m3): ");
    Serial.println(pm25_data.PM_AE_UG_10_0);

    Serial.println();
    return 0;
  }
  return -1;
}


void loop() {
  // call process() to allow background processing, add timeout to avoid high cpu usage
  Serial.print("process(");
  Serial.print(millis());
  Serial.println(")");
  mcs.process(100);

#if (FUNC_SENSOR_DHT22 == 1)
  /* read tempature/hunidity from DHT22 */
  readDHT();
#endif

#if (FUNC_SENSOR_PM25 == 1)
  /* read PM2.5 data from PMS3003 */
  if (readPMS() == 0) {
    /* PM 10 */
    //    MCS_pm10.set(my_aqi.pm10);
    /* PM 2.5 */
    MCS_pm25.set(my_aqi.pm2_5);
    /* PM 1.0 */
    //    MCS_pm01.set(my_aqi.pm1);
  }
#endif

#if (FUNC_SENSOR_CO2 == 1)
  /* read CO2 data form MH-Z16 */
  if (dataRecieve()) {
    Serial.print("  CO2: ");
    Serial.print(my_aqi.co2);
    Serial.println("");
    MCS_co2.set(my_aqi.co2);
  }
#endif

  // check if need to re-connect
  while (!mcs.connected()) {
    Serial.println("re-connect to MCS...");
    mcs.connect();
    if (mcs.connected()) {
      Serial.println("MCS connected !!");
    }
  }

  delay(GET_DATA_INTERVAL);
}

