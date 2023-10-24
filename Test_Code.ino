#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <ArduinoBLE.h>
#define mySerial Serial1

long lat, lon, alti, courseGps;
int day, month, year,SIV, milli, second, minutes, hours,TWA, TWS;
float lat1, lon1, courseGps1, speed,W_Intensity, W_Dir, AWA, AWS;
byte fixType;
uint16_t W_Intensity_raw, W_Dir_raw;
bool Wind;

SFE_UBLOX_GNSS myGNSS;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial)
    ;  //Wait for user to open terminal

  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy failed!");
    while (1)
      ;
  }
  BLE.scanForAddress("d3:82:53:e0:a9:72");


  Serial.println("SparkFun u-blox Example");
  do {
    Serial.println("GNSS: trying 38400 baud");
    mySerial.begin(38400);
    if (myGNSS.begin(mySerial) == true) break;

    delay(100);
    Serial.println("GNSS: trying 9600 baud");
    mySerial.begin(9600);
    if (myGNSS.begin(mySerial) == true) {
      Serial.println("GNSS: connected at 9600 baud, switching to 38400");
      myGNSS.setSerialRate(38400);
      delay(100);
    } else {
      delay(2000);  //Wait a bit before trying again to limit the Serial output
    }
  } while (1);
  Serial.println("GNSS serial connected");

  myGNSS.setUART1Output(COM_TYPE_UBX);  //Set the UART port to output UBX only
  myGNSS.setI2COutput(COM_TYPE_UBX);    //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.setNavigationFrequency(1);
  myGNSS.setAutoPVT(true);
  myGNSS.saveConfiguration();  

  myGNSS.enableDebugging();

  pinMode(86, OUTPUT);
}

void loop() {


  BLEDevice peripheral = BLE.available();
  if (peripheral) {
    Serial.print("Found ");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName());
    Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();

    if (peripheral.localName() != "ULTRASONIC") {
      return;
    }

    BLE.stopScan();
    Ble(peripheral);
    BLE.scanForAddress("d3:82:53:e0:a9:72");
  }
  Main();
}


void Ble(BLEDevice peripheral) {
  // connect to the peripheral
  Serial.println("Connecting ...");

  if (peripheral.connect()) {
    Serial.println("Connected");
  } else {
    Serial.println("Failed to connect!");
    return;
  }
  digitalWrite(87, LOW);
  // discover peripheral attributes
  Serial.println("Discovering attributes ...");
  if (peripheral.discoverAttributes()) {
    Serial.println("Attributes discovered");
  } else {
    Serial.println("Attribute discovery failed!");
    peripheral.disconnect();
    return;
  }

  BLECharacteristic A_W_I_Caratteristic = peripheral.characteristic("2A72");
  BLECharacteristic A_W_D_Caratteristic = peripheral.characteristic("2A73");
  A_W_I_Caratteristic.subscribe();
  A_W_D_Caratteristic.subscribe();

  if (!A_W_I_Caratteristic) {
    peripheral.disconnect();
    return;
  } else if (!A_W_I_Caratteristic.canRead()) {
    peripheral.disconnect();
    return;
  }

  while (peripheral.connected()) {

    if (A_W_I_Caratteristic.valueUpdated()) {
      A_W_I_Caratteristic.readValue(W_Intensity_raw);
      W_Intensity = W_Intensity_raw / 100.0;
    }
    if (A_W_D_Caratteristic.valueUpdated()) {
      A_W_D_Caratteristic.readValue(W_Dir_raw);
      W_Dir = W_Dir_raw / 100;
      Wind = true;
    }
    Main();
  }
}




void Main() {

  if (myGNSS.getPVT()) {
    lat = myGNSS.getLatitude();
    lon = myGNSS.getLongitude();
    alti = myGNSS.getAltitude();
    speed = myGNSS.getGroundSpeed();
    year = myGNSS.getYear();
    month = myGNSS.getMonth();
    day = myGNSS.getDay();
    hours = myGNSS.getHour();
    minutes= myGNSS.getMinute();
    second = myGNSS.getSecond();
    milli = myGNSS.getMillisecond();
    SIV = myGNSS.getSIV();
    courseGps = myGNSS.getHeading();
    //Serial.println(rottaGps*0.00001);
    lat1 = latitudine * 0.0000001;
    lon1 = longitudine * 0.0000001;
    speed = velocita / 1000;
    courseGps1 = rottaGps * 0.00001;
    serial();
    fixType = myGNSS.getFixType();
  }
}



void serial() {
  Serial.print("GPS Course:");
  Serial.println(courseGps1);
  Serial.print("speed:");
  Serial.println(speed);
  Serial.print("AWA:");
  Serial.println(AWA);
  Serial.print("AWS:");
  Serial.println(AWS);
  Serial.print("TWA:");
  Serial.println(TWA);
  Serial.print("TWS:");
  Serial.println(TWS);
  Serial.print("Wind station");
  Serial.println(Wind);
}
