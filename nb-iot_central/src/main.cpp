#include <Arduino.h>
#include <MKRNB.h>

NB nbAccess(true);
NBUDP udp;
GPRS gprs;
char apn[] = "mda.lab5e";               // replace with your APN name
char server_address[] = "172.16.15.14"; // replace with your server's IP address
int server_port = 1234;                 // replace with your server's port number
char pin[] = "1111";

void sendToSpan(char* data, uint8_t data_len)
{
  udp.begin(1232);
  if (udp.beginPacket(server_address, server_port))
  {

    Serial.println("Datalen:");
    Serial.println(data_len);


    Serial.println("Sending UDP packet");
    udp.write(data, data_len);
    //udp.write("Test");
    udp.endPacket();
    Serial.println("Packet sent");
  }
  else
  {
    Serial.println("Unable to send UDP packet");
  }
  udp.stop();
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(115200);
  delay(1000);

  Serial.println("Serial initiated");

  // connection state
  boolean connected = false;
  // Serial.println("AT+CGDCONT=1,\"IP\",\"mda.lab5e\"");

  // After starting the modem with NB.begin()
  // attach to the GPRS network with the APN, login and password
  Serial.println("Tryna connect");
  while (!connected)
  {
    if ((nbAccess.begin(pin, apn) == NB_READY) && (gprs.attachGPRS() == GPRS_READY))
    {
      Serial.println("Attached to gprs");
      connected = true;
    }
    else
    {
      Serial.println("Not connected");
      delay(1000);
    }
  }
  delay(2000);

  Serial.println("Connected to the GPRS network");
}

void loop()
{
  // put your main code here, to run repeatedly:
  if (Serial1.available())
  {
    const uint8_t rx_len = sizeof(uint16_t) + sizeof(uint32_t);
    char rx_buffer[rx_len];

    Serial1.readBytes(rx_buffer,rx_len);
    //String rx_string = Serial1.readString();
    //Serial.println(rx_string);
    sendToSpan(rx_buffer,rx_len);
  }
}