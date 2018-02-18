/*********************************************************************
Code adapted from bleuart sketch.

This code is for the peripheral bluefruit board that turns on the LEDs
and drives the vibration motor.
(loaded onto Drogon)

This was last updated February 15, 2018.
*********************************************************************/

#include <bluefruit.h>

const int ledPin = 7;       // the pin that the LED is attached to
const int motorPin = 16;     // the pin that the motor is attached to
int alarmState = 0;         // 0: safe; 1: doing ok; 2: SOS

BLEClientDis  clientDis;
BLEClientUart clientUart;


void setup()
{
  pinMode(ledPin, OUTPUT);   // initialize the LED as an output
  pinMode(motorPin, OUTPUT); // intitialize the motor as an output
  Serial.begin(115200);
  
  // Initialize Bluefruit with maximum connections as Peripheral = 0, Central = 1
  // SRAM usage required by SoftDevice will increase dramatically with number of connections
  Bluefruit.begin(0, 1);
  
  Bluefruit.setName("Drogon");

  // Configure DIS client
  clientDis.begin();

  // Init BLE Central Uart Serivce
  clientUart.begin();
  clientUart.setRxCallback(bleuart_rx_callback);

  // Increase Blink rate to different from PrPh advertising mode
  Bluefruit.setConnLedInterval(250);

  // Callbacks for Central
  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);

  /* Start Central Scanning
   * - Enable auto scan if disconnected
   * - Interval = 100 ms, window = 80 ms
   * - Don't use active scan
   * - Start(timeout) with timeout = 0 will scan forever (until connected)
   */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80); // in unit of 0.625 ms
  Bluefruit.Scanner.useActiveScan(false);
  Bluefruit.Scanner.start(0);                   // // 0 = Don't stop scanning after n seconds
}

/**
 * Callback invoked when scanner pick up an advertising data
 * @param report Structural advertising data
 */
void scan_callback(ble_gap_evt_adv_report_t* report)
{
  // Check if advertising contain BleUart service
  if ( Bluefruit.Scanner.checkReportForService(report, clientUart) )
  {
    Serial.print("BLE UART service detected. Connecting ... ");

    // Connect to device with bleuart service in advertising
    Bluefruit.Central.connect(report);
  }
}

/**
 * Callback invoked when an connection is established
 * @param conn_handle
 */
void connect_callback(uint16_t conn_handle)
{
  Serial.println("Connected");

  Serial.print("Dicovering DIS ... ");
  if ( clientDis.discover(conn_handle) )
  {
    Serial.println("Found it");
    char buffer[32+1];
    
    // read and print out Manufacturer
    memset(buffer, 0, sizeof(buffer));
    if ( clientDis.getManufacturer(buffer, sizeof(buffer)) )
    {
      Serial.print("Manufacturer: ");
      Serial.println(buffer);
    }

    // read and print out Model Number
    memset(buffer, 0, sizeof(buffer));
    if ( clientDis.getModel(buffer, sizeof(buffer)) )
    {
      Serial.print("Model: ");
      Serial.println(buffer);
    }

    Serial.println();
  }  

  Serial.print("Discovering BLE Uart Service ... ");

  if ( clientUart.discover(conn_handle) )
  {
    Serial.println("Found it");

    Serial.println("Enable TXD's notify");
    clientUart.enableTXD();

    Serial.println("Ready to receive from Khaleesi");
  }else
  {
    Serial.println("Found NONE");
    
    // disconect since we couldn't find bleuart service
    Bluefruit.Central.disconnect(conn_handle);
  }  
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle
 * @param reason
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
  
  Serial.println("Disconnected");
}

/**
 * Callback invoked when uart received data
 * @param uart_svc Reference object to the service where the data 
 * arrived. In this example it is clientUart
 */
void bleuart_rx_callback(BLEClientUart& uart_svc)
  {
    Serial.print("[RX]: ");
  
    String message = "";
    
    while ( uart_svc.available() )
      {
        char incomingChar = (char) uart_svc.read();
        Serial.print( incomingChar );
        message = message + incomingChar;
      }
    
    if (message == "OK")
      {
        alarmState = 1; 
      }
    else if (message == "SOS")
      { 
        alarmState = 2;
      }
    Serial.print("alarmState: ");
    Serial.println(alarmState);

    Serial.println();
  }

void loop()
  {
    if(alarmState == 1) // received OK, turn on LEDs
      {
          digitalWrite(ledPin, HIGH);
          delay(5000);
          alarmState=0;
       }

    else if (alarmState == 2)
      {
        digitalWrite(motorPin, HIGH); // received SOS, turn motor
          delay(5000);
          alarmState=0;
      }
    else
      {
        digitalWrite(ledPin, LOW);
        digitalWrite(motorPin, LOW);
      }
   }

