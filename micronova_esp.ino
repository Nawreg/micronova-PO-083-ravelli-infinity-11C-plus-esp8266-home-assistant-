#include <SoftwareSerial.h>
#include <ArduinoOTA.h>
#include <WiFiManager.h>
#include <PubSubClient.h>

// =================================================================
// CONFIGURATION
// =================================================================
#define mqtt_server "192.168.X.X"
#define mqtt_port 1883
#define mqtt_topic "micronova"
#define mqtt_user "XXXXXXX"
#define mqtt_pass "XXXXXXX"

#define SERIAL_MODE SWSERIAL_8N2
#define RESET_PIN D5
#define RX_PIN D1
#define TX_PIN D4
#define ENABLE_RX D2

SoftwareSerial StoveSerial;
WiFiClient espClient;
PubSubClient client(espClient);
WiFiManager wm;

unsigned long previousMillis = 0;
int lastS = -1;
int lastModState = -1;
int lastExtThermState = -1;
int lastConfClimState = -1;

#define state_topic mqtt_topic "/state"
#define tempset_topic mqtt_topic "/tempset"
#define onoff_topic mqtt_topic "/onoff"
#define ambtemp_topic mqtt_topic "/ambtemp"
#define fumetemp_topic mqtt_topic "/fumetemp"
#define flamepower_topic mqtt_topic "/flamepower"
#define ventilpower_topic mqtt_topic "/ventilpower"
#define pwrget_topic mqtt_topic "/stovepower"
#define in_topic mqtt_topic "/intopic"
#define confort_state_topic mqtt_topic "/confortclimat"
#define thermostat_ext_state_topic mqtt_topic "/extthermostat"
#define modulation_sensor_topic mqtt_topic "/modulation"
#define pong_topic mqtt_topic "/pong"

const char stoveOn[4] = {0x80, 0x20, 0x01, 0xA1};
const char stoveOff[4] = {0x80, 0x20, 0x07, 0xA7};

#define stoveStateAddr 0x20
#define modulationAddr 0x1F
#define ambTempAddr 0xD
#define tempSetAddr 0x53
#define pwrSetAddr 0x52
#define fumesTempAddr 0x9C
#define flamePowerAddr 0x35
#define ventilPowerAddr 0XAD
#define confortClimatAddr 0x5E
#define extThermostatAddr 0x63

char stoveRxData[2];

// --- LOGIQUE SÉRIE ---

void flushSerial() {
    while (StoveSerial.available() > 0) { StoveSerial.read(); }
}

bool checkStoveReply() {
    uint8_t rxCount = 0;
    stoveRxData[0] = 0; 
    stoveRxData[1] = 0;
    unsigned long startMs = millis();
    while (rxCount < 2 && (millis() - startMs < 500)) {
        if (StoveSerial.available()) {
            stoveRxData[rxCount] = StoveSerial.read();
            rxCount++;
        }
    }
    digitalWrite(ENABLE_RX, HIGH);
    return (rxCount == 2); 
}

// --- LECTURES ---

void getModulation() {
    flushSerial();
    StoveSerial.write(0x00); delay(5); StoveSerial.write(modulationAddr);
    digitalWrite(ENABLE_RX, LOW);
    if (checkStoveReply()) {
        uint8_t v = (uint8_t)stoveRxData[1];
        if (v <= 1) {
            lastModState = (int)v;
            client.publish(modulation_sensor_topic, (v == 1 ? "ON" : "OFF"), true);
        }
    }
}

void getStoveState() {
    flushSerial();
    StoveSerial.write(0x00); delay(5); StoveSerial.write(stoveStateAddr); 
    digitalWrite(ENABLE_RX, LOW);
    if (checkStoveReply()) {
        uint8_t s = (uint8_t)stoveRxData[1];
        if (s > 15 && s != 255) return;
        
        lastS = s;
        String stateMsg = ""; 
        String onOffMsg = "ON";

        switch (s) {
            case 0: stateMsg = "Off"; onOffMsg = "OFF"; break;
            case 1: stateMsg = "Demarrage"; break;
            case 2: stateMsg = "Allumage"; break;
            case 3: stateMsg = "??????"; break;
            case 4: stateMsg = "Flamme presente"; break;
            case 5: stateMsg = "Chauffe"; break; // Modulation supprimée d'ici
            case 6: stateMsg = "Nettoyage brasier"; break;
            case 7: stateMsg = "Extinction"; onOffMsg = "OFF"; break;
            case 8: stateMsg = "Eco stop"; onOffMsg = "OFF"; break;
            case 9: stateMsg = "??????"; onOffMsg = "OFF"; break;
            case 10: stateMsg = "Alarme"; onOffMsg = "OFF"; break;
            default: stateMsg = String(s); break;
        }
        client.publish(state_topic, stateMsg.c_str(), true);
        client.publish(onoff_topic, onOffMsg.c_str(), true);
    }
}

void getAmbTemp() { 
    flushSerial(); StoveSerial.write(0x00); delay(5); StoveSerial.write(ambTempAddr); digitalWrite(ENABLE_RX, LOW); 
    if (checkStoveReply()) {
        uint8_t val = (uint8_t)stoveRxData[1]; 
        if (val > 0 && val < 50) client.publish(ambtemp_topic, String(val).c_str(), true);
    }
}

void getFumeTemp() { 
    flushSerial(); StoveSerial.write(0x00); delay(5); StoveSerial.write(fumesTempAddr); digitalWrite(ENABLE_RX, LOW); 
    if (checkStoveReply()) {
        uint8_t val = (uint8_t)stoveRxData[1];
        if (val > 0 && val < 254) client.publish(fumetemp_topic, String(val).c_str(), true);
    }
}

void getTempSet() { 
    flushSerial(); StoveSerial.write(0x20); delay(5); StoveSerial.write(tempSetAddr); digitalWrite(ENABLE_RX, LOW);
    if (checkStoveReply()) {
        uint8_t v = (uint8_t)stoveRxData[1]; 
        if (v > 0) client.publish(tempset_topic, String(v).c_str(), true);
    }
}

void getPwrSet() { 
    flushSerial(); StoveSerial.write(0x20); delay(5); StoveSerial.write(pwrSetAddr); digitalWrite(ENABLE_RX, LOW); 
    if (checkStoveReply()) {
        uint8_t v = (uint8_t)stoveRxData[1]; 
        if (v > 0) client.publish(pwrget_topic, String(v).c_str(), true);
    }
}

void getFlamePower() { 
    flushSerial(); StoveSerial.write(0x00); delay(5); StoveSerial.write(flamePowerAddr); digitalWrite(ENABLE_RX, LOW); 
    if (checkStoveReply()) {
        uint8_t v = (uint8_t)stoveRxData[1]; 
        if (v <= 10) client.publish(flamepower_topic, String(v).c_str(), true);
    }
}

void getVentilPower() { 
    flushSerial(); StoveSerial.write(0x00); delay(5); StoveSerial.write(ventilPowerAddr); digitalWrite(ENABLE_RX, LOW); 
    if (checkStoveReply()) {
        client.publish(ventilpower_topic, String((uint8_t)stoveRxData[1]).c_str(), true);
    }
}

void getConfortClimat() { 
    flushSerial(); StoveSerial.write(0x20); delay(5); StoveSerial.write(confortClimatAddr); digitalWrite(ENABLE_RX, LOW); 
    if (checkStoveReply()) {
        uint8_t v = (uint8_t)stoveRxData[1];
        if (v <= 1 && v != lastConfClimState) {
            lastConfClimState = v;
            client.publish(confort_state_topic, (v == 1 ? "ON" : "OFF"), true);
        }
    }
}

void getExtThermostat() { 
    flushSerial(); StoveSerial.write(0x20); delay(5); StoveSerial.write(extThermostatAddr); digitalWrite(ENABLE_RX, LOW); 
    if (checkStoveReply()) {
        uint8_t v = (uint8_t)stoveRxData[1];
        if (v <= 1 && v != lastExtThermState) {
            lastExtThermState = v;
            client.publish(thermostat_ext_state_topic, (v == 1 ? "ON" : "OFF"), true);
        }
    }
}

void getStates() { 
    getModulation(); delay(300); 
    getStoveState(); delay(300); 
    getTempSet(); delay(300); 
    getPwrSet(); delay(300);
    getAmbTemp(); delay(300); 
    getFumeTemp(); delay(300); 
    getFlamePower(); delay(300); 
    getVentilPower(); delay(300); 
    getConfortClimat(); delay(300); 
    getExtThermostat();
}

// --- CALLBACK MQTT ---

void callback(char *topic, byte *payload, unsigned int length) {
    String msg = "";
    for (unsigned int i = 0; i < length; i++) { msg += (char)payload[i]; }
    int val = msg.toInt();

    if (msg == "ON") { for (int i = 0; i < 4; i++) { StoveSerial.write(stoveOn[i]); delay(5); } }
    else if (msg == "OFF") { for (int i = 0; i < 4; i++) { StoveSerial.write(stoveOff[i]); delay(5); } }
    else if (msg.startsWith("T") || (val >= 7 && val <= 41)) {
        byte tVal = msg.startsWith("T") ? (byte)msg.substring(1).toInt() : (byte)val;
        byte ck = (byte)(0xA0 + 0x53 + tVal);
        StoveSerial.write(0xA0); delay(5); StoveSerial.write(0x53); delay(5); StoveSerial.write(tVal); delay(5); StoveSerial.write(ck);
    }
    else if (msg.startsWith("P") || (val >= 1 && val <= 5)) {
        byte pVal = msg.startsWith("P") ? (byte)msg.substring(1).toInt() : (byte)val;
        byte ck = (byte)(0xA0 + 0x52 + pVal);
        StoveSerial.write(0xA0); delay(5); StoveSerial.write(0x52); delay(5); StoveSerial.write(pVal); delay(5); StoveSerial.write(ck);
    }
    else if (msg.startsWith("C")) {
        byte cVal = (msg.substring(1).toInt() == 1) ? 1 : 0;
        byte ck = (byte)(0xA0 + 0x5E + cVal);
        StoveSerial.write(0xA0); delay(5); StoveSerial.write(0x5E); delay(5); StoveSerial.write(cVal); delay(5); StoveSerial.write(ck);
        delay(600); getConfortClimat();
    }
    else if (msg.startsWith("E")) { 
        byte eVal = (msg.substring(1).toInt() == 1) ? 1 : 0;
        byte ck = (byte)(0xA0 + 0x63 + eVal);
        StoveSerial.write(0xA0); delay(5); StoveSerial.write(0x63); delay(5); StoveSerial.write(eVal); delay(5); StoveSerial.write(ck);
        delay(600); getExtThermostat();
    }
    delay(1500); 
    getStates();
}

// --- SETUP & WIFI ---

void setup_wifi() { 
    ArduinoOTA.setHostname(mqtt_topic); 
    ArduinoOTA.setPassword("micronova"); 
    ArduinoOTA.begin(); 
    WiFi.mode(WIFI_STA); 
    wm.setConnectTimeout(30);
    wm.autoConnect(mqtt_topic);
}

void reconnect() {
    while (!client.connected()) {
        String clientId = "ESPClient-"; clientId += String(random(0xffff), HEX);
        if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
            client.setBufferSize(1024);
            String dev = ",\"dev\":{\"ids\":[\"micronova_controller\"],\"name\":\"micronova controller\",\"mf\":\"Philibert\",\"mdl\":\"Micronova\"}}";
            
            client.publish("homeassistant/sensor/Micronova/State/config", ("{\"uniq_id\":\"micro_state\",\"name\":\"State\",\"stat_t\":\"" + String(state_topic) + "\"" + dev).c_str(), true);
            client.publish("homeassistant/sensor/Micronova/Temperature/config", ("{\"uniq_id\":\"micro_amb_temp\",\"name\":\"Temperature\",\"stat_t\":\"" + String(ambtemp_topic) + "\",\"unit_of_meas\":\"°C\",\"dev_cla\":\"temperature\"" + dev).c_str(), true);
            client.publish("homeassistant/sensor/Micronova/FumesTemperature/config", ("{\"uniq_id\":\"micro_fume_temp\",\"name\":\"Fumes Temperature\",\"stat_t\":\"" + String(fumetemp_topic) + "\",\"unit_of_meas\":\"°C\",\"dev_cla\":\"temperature\"" + dev).c_str(), true);
            client.publish("homeassistant/sensor/Micronova/FlamePower/config", ("{\"uniq_id\":\"micro_flame_v2\",\"name\":\"Flame Power\",\"stat_t\":\"" + String(flamepower_topic) + "\"" + dev).c_str(), true);
            client.publish("homeassistant/sensor/Micronova/VentilPower/config", ("{\"uniq_id\":\"micro_ventil_v2\",\"name\":\"Ventilator Power\",\"stat_t\":\"" + String(ventilpower_topic) + "\"" + dev).c_str(), true);
            client.publish("homeassistant/sensor/Micronova/ModulationSensor/config", ("{\"uniq_id\":\"micro_mod_s\",\"name\":\"Modulation\",\"stat_t\":\"" + String(modulation_sensor_topic) + "\"" + dev).c_str(), true);           
            client.publish("homeassistant/number/Micronova/TargetTemp/config", ("{\"uniq_id\":\"micro_target_t\",\"name\":\"Target Temperature\",\"stat_t\":\"" + String(tempset_topic) + "\",\"cmd_t\":\"" + String(in_topic) + "\",\"min\":7,\"max\":41,\"unit_of_meas\":\"°C\",\"mode\":\"slider\"" + dev).c_str(), true);
            client.publish("homeassistant/number/Micronova/StovePower/config", ("{\"uniq_id\":\"micro_stove_pwr\",\"name\":\"Stove Power\",\"stat_t\":\"" + String(pwrget_topic) + "\",\"cmd_t\":\"" + String(in_topic) + "\",\"min\":1,\"max\":5,\"step\":1,\"mode\":\"box\"" + dev).c_str(), true);
            client.publish("homeassistant/switch/Micronova/Controller/config", ("{\"uniq_id\":\"micro_pow_sw\",\"name\":\"ravelli on off\",\"stat_t\":\"" + String(onoff_topic) + "\",\"cmd_t\":\"" + String(in_topic) + "\",\"payload_on\":\"ON\",\"payload_off\":\"OFF\"" + dev).c_str(), true);
            client.publish("homeassistant/switch/Micronova/ConfortClimat/config", ("{\"uniq_id\":\"micro_conf_sw\",\"name\":\"Confort Climat\",\"stat_t\":\"" + String(confort_state_topic) + "\",\"cmd_t\":\"" + String(in_topic) + "\",\"payload_on\":\"C1\",\"payload_off\":\"C0\",\"state_on\":\"ON\",\"state_off\":\"OFF\"" + dev).c_str(), true);
            client.publish("homeassistant/switch/Micronova/ExtThermostat/config", ("{\"uniq_id\":\"micro_etherm_sw\",\"name\":\"Ext Thermostat\",\"stat_t\":\"" + String(thermostat_ext_state_topic) + "\",\"cmd_t\":\"" + String(in_topic) + "\",\"payload_on\":\"E1\",\"payload_off\":\"E0\",\"state_on\":\"ON\",\"state_off\":\"OFF\"" + dev).c_str(), true);
            
            client.subscribe(in_topic);
            client.publish(pong_topic, "Connected");
        } else {
            delay(5000);
        }
    }
}

void setup() { 
    pinMode(ENABLE_RX, OUTPUT); digitalWrite(ENABLE_RX, HIGH); 
    pinMode(RESET_PIN, INPUT_PULLUP); 
    Serial.begin(115200);
    StoveSerial.begin(1200, SERIAL_MODE, RX_PIN, TX_PIN, false, 256);
    setup_wifi(); 
    client.setServer(mqtt_server, mqtt_port); 
    client.setCallback(callback);
}

void loop() { 
    if (!client.connected()) reconnect(); 
    client.loop(); 
    ArduinoOTA.handle();
    
    if (millis() - previousMillis >= 30000) { 
        previousMillis = millis();
        getStates(); 
        client.publish(pong_topic, "Connected");
    } 
}