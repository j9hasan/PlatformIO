#include <Arduino.h>
#include "lvgl.h"
#include "demos/lv_demos.h"
#include <Arduino_GFX_Library.h>

/*-------------------------------------------------INITIALIZE GRAPHICS SETTINGS----------------------------------------*/
#define GFX_BL DF_GFX_BL
#if defined(ESP32) && (CONFIG_IDF_TARGET_ESP32)

Arduino_DataBus *bus = create_default_Arduino_DataBus();
Arduino_GFX *gfx = new Arduino_ILI9341(bus, DF_GFX_RST, 3 /* rotation */, false /* IPS */);

#endif

#if defined(ESP32) && (CONFIG_IDF_TARGET_ESP32S3)

Arduino_ESP32RGBPanel *bus = new Arduino_ESP32RGBPanel(
    // GFX_NOT_DEFINED /* CS */, GFX_NOT_DEFINED /* SCK */, GFX_NOT_DEFINED /* SDA */,
    41 /* DE */, 40 /* VSYNC */, 39 /* HSYNC */, 42 /* PCLK */,
    14 /* R0 */, 21 /* R1 */, 47 /* R2 */, 48 /* R3 */, 45 /* R4 */,
    9 /* G0 */, 46 /* G1 */, 3 /* G2 */, 8 /* G3 */, 16 /* G4 */, 1 /* G5 */,
    15 /* B0 */, 7 /* B1 */, 6 /* B2 */, 5 /* B3 */, 4 /* B4 */,
    0 /* hsync_polarity */, 180 /* hsync_front_porch */, 30 /* hsync_pulse_width */, 16 /* hsync_back_porch */,
    0 /* vsync_polarity */, 12 /* vsync_front_porch */, 13 /* vsync_pulse_width */, 10 /* vsync_back_porch */ // ,1 /* pclk_active_neg */, 8000000 /* prefer_speed */, GFX_NOT_DEFINED /* auto_flush */

);
Arduino_RGB_Display *gfx = new Arduino_RGB_Display(
    800, 480, bus, 0, true, NULL);

#endif

#include "touch.h"
/*-------------------------------------------------GRAPHICS SETTING DONE----------------------------------------*/

// defines
#define SCAN_INTERVAL 3000
// #define SCAN_COUNT_SLEEP 3
// #define LCD_PWR_PIN 14
// #define ARDUINO_USB_CDC_ON_BOOT 0
// static protootypes
void printMemoryInfo();
void connectToWiFi();
void scanEndedCB(NimBLEScanResults results);

/* Change to your screen resolution */
static uint32_t screenWidth;
static uint32_t screenHeight;
static uint32_t bufSize;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *disp_draw_buf;
static lv_color_t *disp_draw_buf2;
static lv_disp_drv_t disp_drv;

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
#ifndef DIRECT_MODE
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif
#endif // #ifndef DIRECT_MODE

  lv_disp_flush_ready(disp);
}

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
  if (touch_has_signal())
  {
    if (touch_touched())
    {
      data->state = LV_INDEV_STATE_PR;

      /*Set the coordinates*/
      data->point.x = touch_last_x;
      data->point.y = touch_last_y;
    }
    else if (touch_released())
    {
      data->state = LV_INDEV_STATE_REL;
    }
  }
  else
  {
    data->state = LV_INDEV_STATE_REL;
  }
}

// function prototypes
#include <WiFi.h>

const char *ssid = "No internet";
const char *password = "@jamalpur@";

void connectToWiFi()
{
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Attempting to connect to Wi-Fi...");
  }

  Serial.println("Connected to Wi-Fi successfully!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  printMemoryInfo();
}

#include <NimBLEDevice.h>

void scanEndedCB(NimBLEScanResults results);

static NimBLEAdvertisedDevice *advDevice;

static bool doConnect = false;
static uint32_t scanTime = 0; /** 0 = scan forever */

/**  None of these are required as they will be handled by the library with defaults. **
 **                       Remove as you see fit for your needs                        */
class ClientCallbacks : public NimBLEClientCallbacks
{
  void onConnect(NimBLEClient *pClient)
  {
    Serial.println("Connected");
    /** After connection we should change the parameters if we don't need fast response times.
     *  These settings are 150ms interval, 0 latency, 450ms timout.
     *  Timeout should be a multiple of the interval, minimum is 100ms.
     *  I find a multiple of 3-5 * the interval works best for quick response/reconnect.
     *  Min interval: 120 * 1.25ms = 150, Max interval: 120 * 1.25ms = 150, 0 latency, 60 * 10ms = 600ms timeout
     */
    pClient->updateConnParams(120, 120, 0, 60);
  };

  void onDisconnect(NimBLEClient *pClient)
  {
    Serial.print(pClient->getPeerAddress().toString().c_str());
    Serial.println(" Disconnected - Starting scan");
    NimBLEDevice::getScan()->start(scanTime, scanEndedCB);
  };

  /** Called when the peripheral requests a change to the connection parameters.
   *  Return true to accept and apply them or false to reject and keep
   *  the currently used parameters. Default will return true.
   */
  bool onConnParamsUpdateRequest(NimBLEClient *pClient, const ble_gap_upd_params *params)
  {
    if (params->itvl_min < 24)
    { /** 1.25ms units */
      return false;
    }
    else if (params->itvl_max > 40)
    { /** 1.25ms units */
      return false;
    }
    else if (params->latency > 2)
    { /** Number of intervals allowed to skip */
      return false;
    }
    else if (params->supervision_timeout > 100)
    { /** 10ms units */
      return false;
    }

    return true;
  };

  /********************* Security handled here **********************
  ****** Note: these are the same return values as defaults ********/
  uint32_t onPassKeyRequest()
  {
    Serial.println("Client Passkey Request");
    /** return the passkey to send to the server */
    return 123456;
  };

  bool onConfirmPIN(uint32_t pass_key)
  {
    Serial.print("The passkey YES/NO number: ");
    Serial.println(pass_key);
    /** Return false if passkeys don't match. */
    return true;
  };

  /** Pairing process complete, we can check the results in ble_gap_conn_desc */
  void onAuthenticationComplete(ble_gap_conn_desc *desc)
  {
    if (!desc->sec_state.encrypted)
    {
      Serial.println("Encrypt connection failed - disconnecting");
      /** Find the client with the connection handle provided in desc */
      NimBLEDevice::getClientByID(desc->conn_handle)->disconnect();
      return;
    }
  };
};

/** Define a class to handle the callbacks when advertisments are received */
class AdvertisedDeviceCallbacks : public NimBLEAdvertisedDeviceCallbacks
{

  void onResult(NimBLEAdvertisedDevice *advertisedDevice)
  {
    Serial.print("Advertised Device found: ");
    Serial.println(advertisedDevice->toString().c_str());
    if (advertisedDevice->isAdvertisingService(NimBLEUUID("DEAD")))
    {
      Serial.println("Found Our Service");
      /** stop scan before connecting */
      NimBLEDevice::getScan()->stop();
      /** Save the device reference in a global for the client to use*/
      advDevice = advertisedDevice;
      /** Ready to connect now */
      doConnect = true;
    }
  };
};

/** Notification / Indication receiving handler callback */
void notifyCB(NimBLERemoteCharacteristic *pRemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)
{
  std::string str = (isNotify == true) ? "Notification" : "Indication";
  str += " from ";
  /** NimBLEAddress and NimBLEUUID have std::string operators */
  str += std::string(pRemoteCharacteristic->getRemoteService()->getClient()->getPeerAddress());
  str += ": Service = " + std::string(pRemoteCharacteristic->getRemoteService()->getUUID());
  str += ", Characteristic = " + std::string(pRemoteCharacteristic->getUUID());
  str += ", Value = " + std::string((char *)pData, length);
  Serial.println(str.c_str());
}

/** Callback to process the results of the last scan or restart it */
void scanEndedCB(NimBLEScanResults results)
{
  Serial.println("Scan Ended");
}

/** Create a single global instance of the callback class to be used by all clients */
static ClientCallbacks clientCB;

/** Handles the provisioning of clients and connects / interfaces with the server */
bool connectToServer()
{
  NimBLEClient *pClient = nullptr;

  /** Check if we have a client we should reuse first **/
  if (NimBLEDevice::getClientListSize())
  {
    /** Special case when we already know this device, we send false as the
     *  second argument in connect() to prevent refreshing the service database.
     *  This saves considerable time and power.
     */
    pClient = NimBLEDevice::getClientByPeerAddress(advDevice->getAddress());
    if (pClient)
    {
      if (!pClient->connect(advDevice, false))
      {
        Serial.println("Reconnect failed");
        return false;
      }
      Serial.println("Reconnected client");
    }
    /** We don't already have a client that knows this device,
     *  we will check for a client that is disconnected that we can use.
     */
    else
    {
      pClient = NimBLEDevice::getDisconnectedClient();
    }
  }

  /** No client to reuse? Create a new one. */
  if (!pClient)
  {
    if (NimBLEDevice::getClientListSize() >= NIMBLE_MAX_CONNECTIONS)
    {
      Serial.println("Max clients reached - no more connections available");
      return false;
    }

    pClient = NimBLEDevice::createClient();

    Serial.println("New client created");

    pClient->setClientCallbacks(&clientCB, false);
    /** Set initial connection parameters: These settings are 15ms interval, 0 latency, 120ms timout.
     *  These settings are safe for 3 clients to connect reliably, can go faster if you have less
     *  connections. Timeout should be a multiple of the interval, minimum is 100ms.
     *  Min interval: 12 * 1.25ms = 15, Max interval: 12 * 1.25ms = 15, 0 latency, 51 * 10ms = 510ms timeout
     */
    pClient->setConnectionParams(12, 12, 0, 51);
    /** Set how long we are willing to wait for the connection to complete (seconds), default is 30. */
    pClient->setConnectTimeout(5);

    if (!pClient->connect(advDevice))
    {
      /** Created a client but failed to connect, don't need to keep it as it has no data */
      NimBLEDevice::deleteClient(pClient);
      Serial.println("Failed to connect, deleted client");
      return false;
    }
  }

  if (!pClient->isConnected())
  {
    if (!pClient->connect(advDevice))
    {
      Serial.println("Failed to connect");
      return false;
    }
  }

  Serial.print("Connected to: ");
  Serial.println(pClient->getPeerAddress().toString().c_str());
  Serial.print("RSSI: ");
  Serial.println(pClient->getRssi());

  /** Now we can read/write/subscribe the charateristics of the services we are interested in */
  NimBLERemoteService *pSvc = nullptr;
  NimBLERemoteCharacteristic *pChr = nullptr;
  NimBLERemoteDescriptor *pDsc = nullptr;

  pSvc = pClient->getService("DEAD");
  if (pSvc)
  { /** make sure it's not null */
    pChr = pSvc->getCharacteristic("BEEF");

    if (pChr)
    { /** make sure it's not null */
      if (pChr->canRead())
      {
        Serial.print(pChr->getUUID().toString().c_str());
        Serial.print(" Value: ");
        Serial.println(pChr->readValue().c_str());
      }

      if (pChr->canWrite())
      {
        if (pChr->writeValue("Tasty"))
        {
          Serial.print("Wrote new value to: ");
          Serial.println(pChr->getUUID().toString().c_str());
        }
        else
        {
          /** Disconnect if write failed */
          pClient->disconnect();
          return false;
        }

        if (pChr->canRead())
        {
          Serial.print("The value of: ");
          Serial.print(pChr->getUUID().toString().c_str());
          Serial.print(" is now: ");
          Serial.println(pChr->readValue().c_str());
        }
      }

      /** registerForNotify() has been deprecated and replaced with subscribe() / unsubscribe().
       *  Subscribe parameter defaults are: notifications=true, notifyCallback=nullptr, response=false.
       *  Unsubscribe parameter defaults are: response=false.
       */
      if (pChr->canNotify())
      {
        // if(!pChr->registerForNotify(notifyCB)) {
        if (!pChr->subscribe(true, notifyCB))
        {
          /** Disconnect if subscribe failed */
          pClient->disconnect();
          return false;
        }
      }
      else if (pChr->canIndicate())
      {
        /** Send false as first argument to subscribe to indications instead of notifications */
        // if(!pChr->registerForNotify(notifyCB, false)) {
        if (!pChr->subscribe(false, notifyCB))
        {
          /** Disconnect if subscribe failed */
          pClient->disconnect();
          return false;
        }
      }
    }
  }
  else
  {
    Serial.println("DEAD service not found.");
  }

  pSvc = pClient->getService("BAAD");
  if (pSvc)
  { /** make sure it's not null */
    pChr = pSvc->getCharacteristic("F00D");

    if (pChr)
    { /** make sure it's not null */
      if (pChr->canRead())
      {
        Serial.print(pChr->getUUID().toString().c_str());
        Serial.print(" Value: ");
        Serial.println(pChr->readValue().c_str());
      }

      pDsc = pChr->getDescriptor(NimBLEUUID("C01D"));
      if (pDsc)
      { /** make sure it's not null */
        Serial.print("Descriptor: ");
        Serial.print(pDsc->getUUID().toString().c_str());
        Serial.print(" Value: ");
        Serial.println(pDsc->readValue().c_str());
      }

      if (pChr->canWrite())
      {
        if (pChr->writeValue("No tip!"))
        {
          Serial.print("Wrote new value to: ");
          Serial.println(pChr->getUUID().toString().c_str());
        }
        else
        {
          /** Disconnect if write failed */
          pClient->disconnect();
          return false;
        }

        if (pChr->canRead())
        {
          Serial.print("The value of: ");
          Serial.print(pChr->getUUID().toString().c_str());
          Serial.print(" is now: ");
          Serial.println(pChr->readValue().c_str());
        }
      }

      /** registerForNotify() has been deprecated and replaced with subscribe() / unsubscribe().
       *  Subscribe parameter defaults are: notifications=true, notifyCallback=nullptr, response=false.
       *  Unsubscribe parameter defaults are: response=false.
       */
      if (pChr->canNotify())
      {
        // if(!pChr->registerForNotify(notifyCB)) {
        if (!pChr->subscribe(true, notifyCB))
        {
          /** Disconnect if subscribe failed */
          pClient->disconnect();
          return false;
        }
      }
      else if (pChr->canIndicate())
      {
        /** Send false as first argument to subscribe to indications instead of notifications */
        // if(!pChr->registerForNotify(notifyCB, false)) {
        if (!pChr->subscribe(false, notifyCB))
        {
          /** Disconnect if subscribe failed */
          pClient->disconnect();
          return false;
        }
      }
    }
  }
  else
  {
    Serial.println("BAAD service not found.");
  }

  Serial.println("Done with this device!");
  return true;
}

void initBLE()
{
  /** Initialize NimBLE, no device name spcified as we are not advertising */
  NimBLEDevice::init("");

  /** Set the IO capabilities of the device, each option will trigger a different pairing method.
   *  BLE_HS_IO_KEYBOARD_ONLY    - Passkey pairing
   *  BLE_HS_IO_DISPLAY_YESNO   - Numeric comparison pairing
   *  BLE_HS_IO_NO_INPUT_OUTPUT - DEFAULT setting - just works pairing
   */
  // NimBLEDevice::setSecurityIOCap(BLE_HS_IO_KEYBOARD_ONLY); // use passkey
  // NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_YESNO); //use numeric comparison

  /** 2 different ways to set security - both calls achieve the same result.
   *  no bonding, no man in the middle protection, secure connections.
   *
   *  These are the default values, only shown here for demonstration.
   */
  // NimBLEDevice::setSecurityAuth(false, false, true);
  NimBLEDevice::setSecurityAuth(/*BLE_SM_PAIR_AUTHREQ_BOND | BLE_SM_PAIR_AUTHREQ_MITM |*/ BLE_SM_PAIR_AUTHREQ_SC);

  /** Optional: set the transmit power, default is 3db */
#ifdef ESP_PLATFORM
  NimBLEDevice::setPower(ESP_PWR_LVL_P9); /** +9db */
#else
  NimBLEDevice::setPower(9); /** +9db */
#endif

  /** Optional: set any devices you don't want to get advertisments from */
  // NimBLEDevice::addIgnored(NimBLEAddress ("aa:bb:cc:dd:ee:ff"));

  /** create new scan */
  NimBLEScan *pScan = NimBLEDevice::getScan();

  /** create a callback that gets called when advertisers are found */
  pScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());

  /** Set scan interval (how often) and window (how long) in milliseconds */
  pScan->setInterval(45);
  pScan->setWindow(15);

  /** Active scan will gather scan response data from advertisers
   *  but will use more energy from both devices
   */
  pScan->setActiveScan(true);
  /** Start scanning for advertisers for the scan time specified (in seconds) 0 = forever
   *  Optional callback for when scanning stops.
   */
  pScan->start(scanTime, scanEndedCB);

  Serial.println("BLE initialized successfully!");
}

void NumBLETask(void *pv)
{
  while (1)
  {
    /** Loop here until we find a device we want to connect to */
    while (!doConnect)
    {
      delay(1);
    }

    doConnect = false;

    /** Found a device we want to connect to, do it now */
    if (connectToServer())
    {
      Serial.println("Success! we should now be getting notifications, scanning for more!");
    }
    else
    {
      Serial.println("Failed to connect, starting scan");
    }

    NimBLEDevice::getScan()->start(scanTime, scanEndedCB);
  }
}

void setup()
{
  Serial.begin(115200);
  // Serial.setDebugOutput(true);
  // while(!Serial);
  Serial.println("Arduino_GFX LVGL Widgets example");

#ifdef GFX_EXTRA_PRE_INIT
  GFX_EXTRA_PRE_INIT();
#endif

  // Init Display
  if (!gfx->begin())
  {
    Serial.println("gfx->begin() failed!");
  }
  gfx->fillScreen(BLACK);

#ifdef GFX_BL
  pinMode(GFX_BL, OUTPUT);
  digitalWrite(GFX_BL, HIGH);
#endif

  // Init touch device
  touch_init();

  lv_init();

  screenWidth = gfx->width();
  screenHeight = gfx->height();

#ifdef DIRECT_MODE
  bufSize = screenWidth * screenHeight;
#else
  bufSize = screenWidth * 40;
#endif

#ifdef ESP32
  // use MALLOC_CAP_INTERNAL for better perf, but consumes more heap
  disp_draw_buf = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * bufSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  disp_draw_buf2 = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * bufSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!disp_draw_buf)
  {
    // remove MALLOC_CAP_SPIRAM flag try again
    disp_draw_buf = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * bufSize, MALLOC_CAP_8BIT);
  }
#else
  disp_draw_buf = (lv_color_t *)malloc(sizeof(lv_color_t) * bufSize);
#endif
  if (!disp_draw_buf)
  {
    Serial.println("LVGL disp_draw_buf allocate failed!");
  }
  else
  {
    lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, disp_draw_buf2, bufSize);

    /* Initialize the display */
    lv_disp_drv_init(&disp_drv);
    /* Change the following line to your display resolution */
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
#ifdef DIRECT_MODE
    disp_drv.direct_mode = true;
#endif
    lv_disp_drv_register(&disp_drv);

    /* Initialize the (dummy) input device driver */
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);

    lv_demo_widgets();
    printMemoryInfo();
    // call other func
    connectToWiFi();
    // initBLE();
    // xTaskCreatePinnedToCore(NumBLETask, "sendDataTask", 4096, NULL, 1, NULL, 0);

    Serial.println("Setup done");
  }
  printMemoryInfo();
}

void loop()
{
  lv_timer_handler(); /* let the GUI do its work */

#ifdef DIRECT_MODE
#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(0, 0, (uint16_t *)disp_draw_buf, screenWidth, screenHeight);
#else
  gfx->draw16bitRGBBitmap(0, 0, (uint16_t *)disp_draw_buf, screenWidth, screenHeight);
#endif
#endif // #ifdef DIRECT_MODE

#ifdef CANVAS
  gfx->flush();
#endif

  delay(5);
}

void printMemoryInfo()
{
  Serial.print("Free heap: ");
  Serial.print(ESP.getFreeHeap() / 1024);
  Serial.println(" KB");

  Serial.print("Free PSRAM: ");
  Serial.print(ESP.getFreePsram() / 1024);
  Serial.println(" KB");
}
