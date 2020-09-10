#include "Particle.h"

#include "LIS3DH.h"
#include "TinyGPS++.h"
#include "AssetTrackerRK.h" // Only for AssetTracker::antennaExternal

// Example of Wake On Move with the AssetTracker and the Electron
//
// Official project location:
// https://github.com/rickkas7/LIS3DH

// Edited by Austin Stokes


// System threading is required for this project
SYSTEM_THREAD(ENABLED);

// Global objects
FuelGauge batteryMonitor;
LIS3DHSPI accel(SPI, A2, WKP);

TinyGPSPlus gps;


void displayInfo();

// This is the name of the Particle event to publish for battery or movement detection events
// It is a private event.
const char *eventName = "googleDocs";

// Various timing constants
const unsigned long MAX_TIME_TO_PUBLISH_MS = 60000; // Only stay awake for 60 seconds trying to connect to the cloud and publish
const unsigned long MAX_TIME_FOR_GPS_FIX_MS = 180000; // Only stay awake for 3 minutes trying to get a GPS fix
const unsigned long TIME_AFTER_PUBLISH_MS = 4000; // After publish, wait 4 seconds for data to go out
const unsigned long TIME_AFTER_BOOT_MS = 5000; // At boot, wait 5 seconds before going to sleep again (after coming online)
const unsigned long TIME_PUBLISH_BATTERY_SEC = 60 * 60; // Every 60 minutes send a battery update to keep the cellular connection up
const unsigned long TIME_IF_MOVING = 3 * 60; // If the device is moving for 20 seconds then it will go to sleep for 5 minutes
const unsigned long TIME_FOR_SHUTDOWN = 5 * 60; // Time that the device will sleep when manualy shutdown or out of Geo Fence
const unsigned long SERIAL_PERIOD = 5000;
const unsigned long MAX_GPS_AGE_MS = 10000; // GPS location must be newer than this to be considered valid

const uint8_t movementThreshold = 16;

// Lat and Lon for the Geo Fence

float geoFenceLat[] = {45.304314,45.408058,44.742338,44.603158};
float geoFenceLon[] = {-93.795451,-92.682881,-92.832611,-94.051278};
int pnpoly(int nvert, float *vertx, float *verty, float testx, float testy);

// Stuff for the finite state machine
enum State { ONLINE_WAIT_STATE, RESET_STATE, RESET_WAIT_STATE, PUBLISH_STATE, SLEEP_STATE, SLEEP_WAIT_STATE, CALIBRATE_STATE, BOOT_WAIT_STATE, GPS_WAIT_STATE, SHUT_DOWN_STATE };
State state = ONLINE_WAIT_STATE;
unsigned long stateTime = 0;
int awake = 0;
unsigned long lastSerial = 0;
unsigned long startFix = 0;
bool gettingFix = false;
int powerToScooter = 100;

int inGeoFence = 0;
int aliveState = 1;
int forceSleep(String value);

PRODUCT_ID(8957);
PRODUCT_VERSION(4);


void setup() {
	Serial.begin(9600);

	// The GPS module on the AssetTracker is connected to Serial1 and D6
	Serial1.begin(9600);

	// Settings D6 LOW powers up the GPS module
    pinMode(D6, OUTPUT);
    digitalWrite(D6, LOW);
    startFix = millis();
    gettingFix = true;

    // To use an external antenna, uncomment this line:
    { AssetTracker t; t.antennaExternal(); }

	Particle.function("kill",forceSleep);
	Particle.variable("getAliveState", &aliveState, INT);

	System.enableUpdates();
}


void loop() {
	while (Serial1.available() > 0) {
		if (gps.encode(Serial1.read())) {
			displayInfo();
		}
	}

	switch(state) {
	case ONLINE_WAIT_STATE:
		if (Particle.connected()) {
			state = RESET_STATE;
		}
		if (millis() - stateTime > 5000) {
			stateTime = millis();
			Serial.println("waiting to come online");
		}
		break;

	case RESET_STATE: {
		Serial.println("resetting accelerometer");

		LIS3DHConfig config;
		config.setLowPowerWakeMode(16);

		if (!accel.setup(config)) {
			Serial.println("accelerometer not found");
			state = SLEEP_STATE;
			break;
		}

		state = BOOT_WAIT_STATE;
		}
		break;

	case GPS_WAIT_STATE:
		if (gps.location.isValid() && gps.location.age() < MAX_GPS_AGE_MS) {
			// Got a GPS fix
			state = PUBLISH_STATE;
			break;
		}
		if (millis() - stateTime >= MAX_TIME_FOR_GPS_FIX_MS) {
			Serial.println("failed to get GPS fix");
			state = SLEEP_STATE;
			break;

		}
		break;

	case PUBLISH_STATE:
		if (Particle.connected()) {
			// The publish data contains 5 values:
			// latitude (decimal)
			// logitude (decimal)
			// state of charge (decimal)
			// whether movement was detected (1) or not (0) The not detected publish is used for battery status updates
			// alive state (1) if alive (0) if not
			char data[256];
			float stateOfCharge = batteryMonitor.getSoC();
      		snprintf(data, sizeof(data), "{\"La\":\"%f\",\"Lo\":\"%f\",\"C\":\"%.02f\",\"A\":\"%d\",\"S\":\"%d\"}",
          	gps.location.lat(),gps.location.lng(), stateOfCharge, awake, aliveState);

			Particle.publish(eventName, data, 60, PRIVATE);
			Serial.println(data);

			inGeoFence = pnpoly((sizeof(geoFenceLat)/sizeof(geoFenceLat[0])),geoFenceLat,geoFenceLon,gps.location.lat(),gps.location.lng());

			// Wait for the publish to go out
			stateTime = millis();
			state = SLEEP_WAIT_STATE;
		}
		else {
			// Haven't come online yet
			if (millis() - stateTime >= MAX_TIME_TO_PUBLISH_MS) {
				// Took too long to publish, just go to sleep
				state = SLEEP_STATE;
			}
		}
		break;

	case SLEEP_WAIT_STATE:
		if (millis() - stateTime >= TIME_AFTER_PUBLISH_MS) {
			state = SLEEP_STATE;
		}
		break;

	case BOOT_WAIT_STATE:
		if (millis() - stateTime >= TIME_AFTER_BOOT_MS) {
			// To publish the battery stats after boot, set state to PUBLISH_STATE
			// To go to sleep immediately, set state to SLEEP_STATE
			state = GPS_WAIT_STATE;
			stateTime = millis();
		}
		break;

	case SLEEP_STATE:
		// It's a good idea to reset the accelerometer here. It shouldn't be necessary, but 
		// sometimes if you don't do this, the Electron will never wake up again. This is
		// oddly correlated with powering down the GPS
		{
			LIS3DHConfig config;
			config.setLowPowerWakeMode(16);
	
			accel.setup(config);
		}
		if(!aliveState || !inGeoFence){
			state = SHUT_DOWN_STATE;
		}
		else{
			powerToScooter = 100;
			state = CALIBRATE_STATE;
		}
		stateTime = millis();
		break;

	case SHUT_DOWN_STATE:
		Serial.println("Invalid");
		// If the aliveState is off then slowly lower power to scooter
		if(powerToScooter>0 && !aliveState){
			powerToScooter--;
			delay(1000);
			state = SHUT_DOWN_STATE;
			stateTime = millis();
		}
		// If the scooter is not in the Geo Fence then slowly lower power to scooter
		// and check if the device gets back in Geo Fence
		else if(powerToScooter>0 && !inGeoFence){
			powerToScooter--;
			if(powerToScooter%10==0){
				if (gps.location.isValid() && gps.location.age() < MAX_GPS_AGE_MS) {
					inGeoFence = pnpoly((sizeof(geoFenceLat)/sizeof(geoFenceLat[0])),geoFenceLat,geoFenceLon,gps.location.lat(),gps.location.lng());
				}
			}
			delay(1000);
			state = SHUT_DOWN_STATE;
			stateTime = millis();
		}
		else if(aliveState && inGeoFence){
			powerToScooter=100;

			Serial.printlnf("awake=%d", awake);

			// Restart the GPS
			digitalWrite(D6, LOW);
			startFix = millis();
			gettingFix = true;

			state = GPS_WAIT_STATE;
			stateTime = millis();
		}
		else{

			Serial.println("Going Into Invalid Sleep");
			delay(500);

			System.sleep(D1, RISING, TIME_FOR_SHUTDOWN, SLEEP_NETWORK_STANDBY);

			delay(500);

			awake = 0;

			Serial.printlnf("awake=%d", awake);

			// Restart the GPS
			digitalWrite(D6, LOW);
			startFix = millis();
			gettingFix = true;

			state = GPS_WAIT_STATE;
			stateTime = millis();
		}
		Serial.print("Power to scooter: ");
		Serial.println(powerToScooter);
		break;

	case CALIBRATE_STATE:
		LIS3DHConfig config;
		config.setLowPowerWakeMode(16);
	
		accel.setup(config);
		// Wait for Electron to stop moving for 20 seconds so we can recalibrate the accelerometer
		// Wait for 40 seconds before looping again
		if (accel.calibrateFilter(20000, 40000)) {
			// We've stopped moving and the accelerometer is calibrated

			// Uncomment this line to power down the GPS. It saves power but will increase the amount
			// of time to get a fix.
			digitalWrite(D6, HIGH);

			Serial.println("Checking for Sleep");
			delay(500);

			// If you use SLEEP_MODE_DEEP it's very important to make sure WKP is LOW before going to
			// sleep. If you go into SLEEP_MODE_DEEP with WKP high you will likely never wake up again
			// (until reset).
			if (digitalRead(WKP)) {
				// Try to calibrate again
				break;
			}

			if(!awake){
				// Sleep
				Serial.println("Going Into Soft Sleep");
				System.sleep(WKP, RISING, TIME_PUBLISH_BATTERY_SEC, SLEEP_NETWORK_STANDBY);
			}

			// This delay should not be necessary, but sometimes things don't seem to work right
			// immediately coming out of sleep.
			delay(500);

			awake = ((accel.clearInterrupt() & LIS3DH::INT1_SRC_IA) != 0);
			if(awake){
				System.disableUpdates();
			}
			else{
				System.enableUpdates();
			}

			Serial.printlnf("awake=%d", awake);

			// Restart the GPS
			digitalWrite(D6, LOW);
			startFix = millis();
			gettingFix = true;

			state = GPS_WAIT_STATE;
			stateTime = millis();
		}
		else {
			digitalWrite(D6, HIGH);

			Serial.println("Going Into Deep Sleep");
			delay(500);

			Serial.printlnf("still moving after %u sec", (millis() - stateTime) / 1000);
			System.sleep(D1, RISING, TIME_IF_MOVING, SLEEP_NETWORK_STANDBY);

			delay(500);

			awake = 1;

			Serial.printlnf("awake=%d", awake);

			// Restart the GPS
			digitalWrite(D6, LOW);
			startFix = millis();
			gettingFix = true;

			state = GPS_WAIT_STATE;
			stateTime = millis();
		}
		break;
	}
}



void displayInfo()
{
	if (millis() - lastSerial >= SERIAL_PERIOD) {
		lastSerial = millis();

		char buf[128];
		if (gps.location.isValid()) {
			snprintf(buf, sizeof(buf), "%f,%f", gps.location.lat(), gps.location.lng());
			if (gettingFix) {
				gettingFix = false;
				unsigned long elapsed = millis() - startFix;
				Serial.printlnf("%lu milliseconds to get GPS fix", elapsed);
			}
		}
		else {
			strcpy(buf, "no location");
			if (!gettingFix) {
				gettingFix = true;
				startFix = millis();
			}
		}
		Serial.println(buf);
    Serial.print("In the Geo Fence: ");
    Serial.println(pnpoly((sizeof(geoFenceLat)/sizeof(geoFenceLat[0])),geoFenceLat,geoFenceLon,gps.location.lat(),gps.location.lng()));
	Serial.print("Alive State: ");
	Serial.println(aliveState);
	}
}

// check if testx and testy are in the polygon defined by vertx and verty with nvert verticies
int pnpoly(int nvert, float *vertx, float *verty, float testx, float testy)
{
  int i, j, c = 0;
  for (i = 0, j = nvert-1; i < nvert; j = i++) {
    if ( ((verty[i]>testy) != (verty[j]>testy)) &&
	 (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
       c = !c;
  }
  return c;
}

int forceSleep(String value)
{
	aliveState = value.toInt();
	Serial.println(aliveState);
	return aliveState;
}