#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <WiFiUDP.h>
#include <EEPROM.h>

#define DEFAULT_SSID_LENGTH 16

#define LED 16
#define Print_Debug

typedef struct
{
  int8_t startByte;
  int8_t roll;
  int8_t pitch;
  int8_t throttle;
  int8_t yaw;
  int8_t checksum;
  char ssid[DEFAULT_SSID_LENGTH];
} ControlData;

typedef struct
{
  int8_t startByte;
  int8_t startByte2;
  int8_t yawPitchRoll;
  int16_t kp;
  int16_t ki;
  int16_t kd;
  int16_t checksum;
  char ssid[DEFAULT_SSID_LENGTH];
} TuningData;

WiFiUDP udp;
byte data[512] = {0};
unsigned int localPort = 12345;
TuningData tuningData[3] = {0};

#ifdef Print_Debug    
String yawPitchRollText[3] = {"Yaw....:", "Pitch..:", "Roll...:"};
String output = "";
String lastOutput = "";
#endif

char accessPointName[DEFAULT_SSID_LENGTH] = {'\0'};
Ticker ticker_DEGUG;


String readEEPROM(int index, int length);
int writeEEPROM(int index, String text);
void loadTuningData();
void saveTuningData(int i);
String ipToString(IPAddress ip);
String floatToString(float value, int length, int decimalPalces);
String intToString(int value, int length);
String hexToString(byte value);
void Debug(void);
void Read_udp(void);


IPAddress local_ip(192, 168, 5, 1);
IPAddress gateway(192, 168, 5, 1);
IPAddress subnet(255, 255, 255, 255);

void setup()
{
  WiFi.mode(WIFI_STA);
  delay(50);
  
  WiFi.disconnect();
  delay(200);

  pinMode(LED, OUTPUT);
  delay(200);

  Serial.begin(115200);
  sprintf(accessPointName, "B-Drone-%lu", ESP.getChipId());
  WiFi.softAP(accessPointName, "12345678");
  delay(50);
  accessPointName[DEFAULT_SSID_LENGTH-1] = {'\0'};


  WiFi.mode(WIFI_AP_STA);
  delay(50);

  WiFi.softAPConfig(local_ip, gateway, subnet);
  delay(50);

  udp.begin(localPort);
  delay(50);

#ifdef Print_Debug

	Serial.println("");
  Serial.print("RemoteTest Access Point: ");
  Serial.println(accessPointName);
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("UDP Port: ");
  Serial.println(localPort);
  Serial.println();

#endif

  EEPROM.begin(512);
  delay(50);

  loadTuningData();

  ticker_DEGUG.attach_ms(500, Debug);
}

void loop()
{
  delay(5);
  Read_udp();
}

void Read_udp(void)
{
  int numberOfBytes = udp.parsePacket();

  if (numberOfBytes > 0)
  {

    udp.read(data, numberOfBytes);

   

    if (data[0] == 0XF0 && data[1] == 0XF0) // tuning data
    {
      TuningData tuningDataBuffer = {0};
      memcpy(&tuningDataBuffer, data, sizeof(TuningData));
      int16_t checksum = tuningDataBuffer.yawPitchRoll + tuningDataBuffer.kp + tuningDataBuffer.ki + tuningDataBuffer.kd;
      tuningDataBuffer.ssid[DEFAULT_SSID_LENGTH - 1] = '\0';

      
      if (tuningDataBuffer.yawPitchRoll >= 0X01 && tuningDataBuffer.yawPitchRoll <= 0X03 && tuningDataBuffer.checksum == checksum && strcmp(tuningDataBuffer.ssid, accessPointName) == 0)
      {
        int i = tuningDataBuffer.yawPitchRoll - 1;
        tuningData[i] = tuningDataBuffer;
        saveTuningData(i);

        float kp = tuningDataBuffer.kp / 10.0f;
        float ki = tuningDataBuffer.ki / 10.0f;
        float kd = tuningDataBuffer.kd / 10.0f;

#ifdef Print_Debug

			Serial.print("startByte:");Serial.print(tuningDataBuffer.startByte,HEX);
			Serial.print("	startByte2:");Serial.println(tuningDataBuffer.startByte2,HEX);
			Serial.print("	yawPitchRoll:");Serial.print(yawPitchRollText[tuningDataBuffer.yawPitchRoll-1]);
			Serial.print("	kp:");Serial.print(kp);
			Serial.print("	ki:");Serial.print(ki);
			Serial.print("	kd:");Serial.println(kd);
			Serial.print("	checksum:");Serial.println(tuningDataBuffer.checksum);
			Serial.print("	check ssid:");Serial.println(strcmp(tuningDataBuffer.ssid, accessPointName) == 0);

    //     if (i == 0)
    //     {
    //     	String string_tmp_0 = "";
    //       string_tmp_0 += String("Receive tuning data from ") + ipToString(udp.remoteIP()) + " port " + udp.remotePort() + ":";
    //       Serial.println(string_tmp_0);
    //     }

				// String string_tmp_1 = "";
    //     string_tmp_1 = String(yawPitchRollText[i]) + " KP " + floatToString(kp, 5, 1) + ", KI " + floatToString(ki, 5, 1) + ", KD " + floatToString(kd, 5, 1);
    //     Serial.println(string_tmp_1
#endif


      }
    }
    else if (data[0] == 0XFC && data[1] == 0XFC) // get tuning data
    {

      TuningData tuningDataBuffer = {0};
      memcpy(&tuningDataBuffer, data, sizeof(TuningData));
      int16_t checksum = tuningDataBuffer.yawPitchRoll + tuningDataBuffer.kp + tuningDataBuffer.ki + tuningDataBuffer.kd;
      tuningDataBuffer.ssid[DEFAULT_SSID_LENGTH - 1] = '\0';

      if (tuningDataBuffer.checksum == checksum && strcmp(tuningDataBuffer.ssid, accessPointName) == 0)
      {
        // send tuning data to remote control
        for (tuningDataBuffer.yawPitchRoll = 1; tuningDataBuffer.yawPitchRoll <= 3; tuningDataBuffer.yawPitchRoll++)
        {
          int i = tuningDataBuffer.yawPitchRoll - 1;
          tuningDataBuffer.kp = tuningData[i].kp;
          tuningDataBuffer.ki = tuningData[i].ki;
          tuningDataBuffer.kd = tuningData[i].kd;
          tuningDataBuffer.checksum = tuningDataBuffer.yawPitchRoll + tuningDataBuffer.kp + tuningDataBuffer.ki + tuningDataBuffer.kd;
          memcpy(&data, &tuningDataBuffer, sizeof(TuningData));
				
#ifdef Print_Debug



          if (i == 0)
          {
          	String string_tmp_3 = "";
            string_tmp_3 += String("Send tuning data to ") + ipToString(udp.remoteIP()) + " port " + udp.remotePort() + ":";
            Serial.println(string_tmp_3);
          }

          String string_tmp_4 = "";
          string_tmp_4 = hexToString(tuningDataBuffer.startByte) + ", ";
          string_tmp_4 += hexToString(tuningDataBuffer.startByte2) + ", ";
          string_tmp_4 += hexToString(tuningDataBuffer.yawPitchRoll) + ", ";
          string_tmp_4 += intToString(tuningDataBuffer.kp, 4) + ", " + intToString(tuningDataBuffer.ki, 4) + ", " + intToString(tuningDataBuffer.kd, 4) + ", " + intToString(tuningDataBuffer.checksum, 4);
          Serial.println(string_tmp_4);

#endif

          udp.beginPacket(udp.remoteIP(), udp.remotePort());
          udp.write(data, sizeof(TuningData));
          udp.endPacket();
          delay(200); // don't forget to delay
        }
      }
    }
    else if (data[0] == 0XFE) // trim and control
    {

      ControlData controlData = {0};
      memcpy(&controlData, data, sizeof(ControlData));

#ifdef Print_Debug
// typedef struct
// {
//   int8_t startByte;
//   int8_t roll;
//   int8_t pitch;
//   int8_t throttle;
//   int8_t yaw;
//   int8_t checksum;
//   char ssid[DEFAULT_SSID_LENGTH];
// } ControlData;

			Serial.print("startByte:");Serial.print(controlData.startByte,HEX);
			Serial.print("	roll:");Serial.print(controlData.roll);
			Serial.print("	pitch:");Serial.print(controlData.pitch);
			Serial.print("	throttle:");Serial.print(controlData.throttle);
			Serial.print("	yaw:");Serial.println(controlData.yaw);
			Serial.print("	checksum:");Serial.println(controlData.checksum);
			Serial.print("	check ssid:");Serial.println(strcmp(controlData.ssid, accessPointName) == 0);
#endif

      int8_t checksum = controlData.roll + controlData.pitch + controlData.throttle + controlData.yaw;
      controlData.ssid[DEFAULT_SSID_LENGTH - 1] = '\0';

      if (controlData.checksum == checksum && strcmp(controlData.ssid, accessPointName) == 0)
      {
        int8_t trimFlag = 0XFF;

        if (controlData.roll == trimFlag && controlData.pitch == trimFlag && controlData.throttle == trimFlag && controlData.yaw == trimFlag)
        {



#ifdef Print_Debug        	
          Serial.println("Trim");
#endif          
        }
        else
        {

#ifdef Print_Debug
          output = String("Roll ") + intToString(controlData.roll, 4) + ", Pitch " + intToString(controlData.pitch, 4) + ", Throttle " + intToString(controlData.throttle, 3) + ", Yaw " + intToString(controlData.yaw, 4);

          if (output != lastOutput)
          {
            lastOutput = output;
            Serial.println(output);
          }
#endif
        }
      }

    }
  }
}

String readEEPROM(int index, int length)
{
  String text = "";
  char ch = 1;

  for (int i = index; (i < (index + length)) && ch; ++i)
  {
    if (ch = EEPROM.read(i))
    {
      text.concat(ch);
    }
  }

  return text;
}

int writeEEPROM(int index, String text)
{
  for (int i = index; i < text.length() + index; ++i)
  {
    EEPROM.write(i, text[i - index]);
  }

  EEPROM.write(index + text.length(), 0);
  EEPROM.commit();

  return text.length() + 1;
}

void loadTuningData()
{
  for (int i = 0; i < 3; i++)
  {
    int address = i * 12;

    String str = readEEPROM(address, 4);
    tuningData[i].kp = str.toInt();
    address += 4;

    str = readEEPROM(address, 4);
    tuningData[i].ki = str.toInt();
    address += 4;

    str = readEEPROM(address, 4);
    tuningData[i].kd = str.toInt();
  }
}

void saveTuningData(int i)
{
  int address = i * 12;

  String str = String(tuningData[i].kp);
  writeEEPROM(address, str);
  address += 4;

  str = String(tuningData[i].ki);
  writeEEPROM(address, str);
  address += 4;

  str = String(tuningData[i].kd);
  writeEEPROM(address, str);
}

String ipToString(IPAddress ip)
{
  return String(ip[0]) + "." + ip[1] + "." + ip[2] + "." + ip[3];
}

String floatToString(float value, int length, int decimalPalces)
{
  String stringValue = String(value, decimalPalces);
  String prefix = "";

  for (int i = 0; i < length - stringValue.length(); i++)
  {
    prefix += " ";
  }

  return prefix + stringValue;
}

String intToString(int value, int length)
{
  String stringValue = String(value);
  String prefix = "";

  for (int i = 0; i < length - stringValue.length(); i++)
  {
    prefix += " ";
  }

  return prefix + stringValue;
}

String hexToString(byte value)
{
  int length = 2;
  String stringValue = String(value, HEX);
  String prefix = "";

  for (int i = 0; i < length - stringValue.length(); i++)
  {
    prefix += "0";
  }

  return "0x" + prefix + stringValue;
}

void Debug(void)
{
  static int8_t led = 0;
  led = 1 - led;
  digitalWrite(LED, led);
}
