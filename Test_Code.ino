#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <ArduinoBLE.h>
#define mySerial Serial1

long latitudine, longitudine, altitudine, rottaGps;
int giorno, mese, anno;
int milli, secondi, minuti, ore, deltaHeading, TWA, TWS;
float latitudine1, longitudine1, rottaGps1;
byte fixType;
int SIV;
uint16_t W_Intensity_raw;
float W_Intensity;
float AWS;
uint16_t W_Dir_raw;
float W_Dir;
float AWA;

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
      //myGNSS.factoryReset();
      delay(2000);  //Wait a bit before trying again to limit the Serial output
    }
  } while (1);
  Serial.println("GNSS serial connected");

  myGNSS.setUART1Output(COM_TYPE_UBX);  //Set the UART port to output UBX only
  myGNSS.setI2COutput(COM_TYPE_UBX);    //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.setNavigationFrequency(1);
  myGNSS.setAutoPVT(true);
  myGNSS.saveConfiguration();  //Save the current settings to flash and BBR
  //myGNSS.powerSaveMode();
  myGNSS.enableDebugging();

  pinMode(86, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  BLEDevice peripheral = BLE.available();
  if (peripheral) {
    // discovered a peripheral, print out address, local name, and advertised service
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
    latitudine = myGNSS.getLatitude();
    longitudine = myGNSS.getLongitude();
    altitudine = myGNSS.getAltitude();
    velocita = myGNSS.getGroundSpeed();
    anno = myGNSS.getYear();
    mese = myGNSS.getMonth();
    giorno = myGNSS.getDay();
    ore = myGNSS.getHour();
    minuti = myGNSS.getMinute();
    secondi = myGNSS.getSecond();
    milli = myGNSS.getMillisecond();
    SIV = myGNSS.getSIV();
    rottaGps = myGNSS.getHeading();
    //Serial.println(rottaGps*0.00001);
    latitudine1 = latitudine * 0.0000001;
    longitudine1 = longitudine * 0.0000001;
    velocita = velocita / 1000;
    rottaGps1 = rottaGps * 0.00001;
    seriale();
    fixType = myGNSS.getFixType();
  }
}



void seriale() {
  Serial.print("rotta gps:");
  Serial.println(rottaGps1);
  Serial.print("velocità");
  Serial.println(velocita);
  Serial.print("direzione vento apparente:");
  Serial.println(AWA);
  Serial.print("intensità vento apparente:");
  Serial.println(AWS);
  Serial.print("direzione vento reale:");
  Serial.println(TWA);
  Serial.print("intensità vento reale:");
  Serial.println(TWS);

  Serial.println(Wind);
}
