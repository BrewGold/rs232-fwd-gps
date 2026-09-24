// HISTORICAL NOTE: ESP32-S3-specific sketch kept only for reference.
// ======================================================
// RS232 FWD GPS - DRAFT V0.9
//
// ESP32-S3
// RTK3B Budget (UM980)
// BNO085
// Dynatest
//
// TODO:
// - Parser definitivo PPPNAVA
// - Offset real antena -> pistón
// - Altura real antena
// ======================================================

#include <Arduino.h>
#include <Wire.h>

// ======================================================
// CONFIGURACION
// ======================================================

#define GNSS_BAUD 115200
#define DYNATEST_BAUD 38400

#define GNSS_RX_PIN 44
#define GNSS_TX_PIN 43

#define OUT_RX_PIN 18
#define OUT_TX_PIN 17

#define SDA_PIN 8
#define SCL_PIN 9

#define LED_POWER 4
#define LED_GNSS  5

const float OFFSET_METERS = 1.50f;     // TODO
const float ANTENNA_HEIGHT = 0.0f;     // TODO

const uint32_t STOP_CONFIRM_MS = 2000;
const uint32_t AVG_TIME_MS     = 15000;
const uint32_t GGA_PERIOD_MS   = 100;

const float STOP_ENTER_SPEED = 0.20f;
const float STOP_EXIT_SPEED  = 0.30f;

const float NEW_LOCATION_DIST = 1.0f;

// ======================================================
// UART
// ======================================================

HardwareSerial GNSS(1);
HardwareSerial DYN(2);

// ======================================================
// ESTADOS
// ======================================================

enum PppState
{
    SIN_PPP,
    PPP_CONVERGING,
    PPP_ESTABLE,
    PPP_DESCONOCIDO
};

enum PositionState
{
    MOVING,
    STOP_CONFIRM,
    AVERAGING,
    LOCKED
};

PppState pppState = SIN_PPP;
PositionState posState = MOVING;

// ======================================================
// DATOS GNSS
// ======================================================

struct Sample
{
    double lat;
    double lon;
    double alt;
    double yaw;
};

static const int MAX_SAMPLES = 160;

Sample samples[MAX_SAMPLES];
int sampleCount = 0;

double currentLat = NAN;
double currentLon = NAN;
double currentAlt = NAN;

double currentYaw = NAN;

double speedMS = 0.0;

String utcTime;

bool gnssValid = false;

// ======================================================
// LOCKED
// ======================================================

double latLocked = NAN;
double lonLocked = NAN;
double altLocked = NAN;

bool lockedValid = false;

// ======================================================
// TODO BNO085
// ======================================================

bool readYaw(double &yaw)
{
    // TODO:
    // Sustituir por lectura real BNO085
    yaw = 0.0;
    return true;
}

// ======================================================
// TODO PPPNAV
// ======================================================

void parsePPPNAV(const String& line)
{
    if(!line.startsWith("#PPPNAVA"))
        return;

    Serial.print("[PPPNAV RAW] ");
    Serial.println(line);

    if(line.indexOf("PPP_CONVERGING") >= 0)
    {
        pppState = PPP_CONVERGING;
        return;
    }

    if(line.indexOf(",PPP,") >= 0)
    {
        pppState = PPP_ESTABLE;
        return;
    }

    if(line.indexOf("SINGLE") >= 0)
    {
        pppState = SIN_PPP;
        return;
    }

    pppState = PPP_DESCONOCIDO;
}

// ======================================================
// FIXQ SALIDA
// ======================================================

uint8_t outputFixQ()
{
    switch(pppState)
    {
        case PPP_ESTABLE:
            return 4;

        case PPP_CONVERGING:
        case SIN_PPP:
        case PPP_DESCONOCIDO:
            return 2;

        default:
            return 0;
    }
}

// ======================================================
// GGA
// ======================================================

String buildGPGGA(
    const String& utc,
    double lat,
    double lon,
    double alt,
    uint8_t fixQ,
    uint8_t sats)
{
    // TODO:
    // reutilizar tu función existente
    return "$GPGGA,...";
}

// ======================================================
// MEDIA RECORTADA
// ======================================================

double trimmedMean(double* data, int n)
{
    if(n <= 0) return NAN;

    std::sort(data, data + n);

    int trim = n * 0.05;

    if(trim * 2 >= n)
        trim = 0;

    double sum = 0.0;
    int cnt = 0;

    for(int i = trim; i < n-trim; i++)
    {
        sum += data[i];
        cnt++;
    }

    if(cnt == 0)
        return NAN;

    return sum / cnt;
}

// ======================================================
// LOCK
// ======================================================

void createLockedPosition()
{
    double lat[MAX_SAMPLES];
    double lon[MAX_SAMPLES];
    double alt[MAX_SAMPLES];
    double yaw[MAX_SAMPLES];

    for(int i=0;i<sampleCount;i++)
    {
        lat[i] = samples[i].lat;
        lon[i] = samples[i].lon;
        alt[i] = samples[i].alt;
        yaw[i] = samples[i].yaw;
    }

    double latMean = trimmedMean(lat,sampleCount);
    double lonMean = trimmedMean(lon,sampleCount);
    double altMean = trimmedMean(alt,sampleCount);
    double yawMean = trimmedMean(yaw,sampleCount);

    // ==================================================
    // TODO OFFSET
    // ==================================================
    //
    // aplicar offset antena->pistón
    // utilizando yawMean
    //
    // por ahora:
    //
    latLocked = latMean;
    lonLocked = lonMean;
    altLocked = altMean;

    lockedValid = true;

    Serial.println("LOCKED");
}

// ======================================================
// DISTANCIA
// ======================================================

double distanceMeters(
    double lat1,
    double lon1,
    double lat2,
    double lon2)
{
    const double R = 6378137.0;

    double dLat =
        radians(lat2-lat1);

    double dLon =
        radians(lon2-lon1);

    double a =
        sin(dLat/2.0)*sin(dLat/2.0)
      + cos(radians(lat1))
      * cos(radians(lat2))
      * sin(dLon/2.0)
      * sin(dLon/2.0);

    double c =
        2.0 * atan2(sqrt(a),sqrt(1.0-a));

    return R*c;
}

// ======================================================
// SETUP
// ======================================================

void setup()
{
    Serial.begin(115200);

    pinMode(LED_POWER,OUTPUT);
    pinMode(LED_GNSS,OUTPUT);

    digitalWrite(LED_POWER,HIGH);

    GNSS.begin(
        GNSS_BAUD,
        SERIAL_8N1,
        GNSS_RX_PIN,
        GNSS_TX_PIN);

    DYN.begin(
        DYNATEST_BAUD,
        SERIAL_8N1,
        OUT_RX_PIN,
        OUT_TX_PIN);

    Wire.begin(SDA_PIN,SCL_PIN);

    Serial.println("RS232 FWD GPS V0.9 Draft");
}

// ======================================================
// LOOP
// ======================================================

void loop()
{
    // ------------------------------------------
    // TODO
    // Parse GGA
    // Parse RMC
    // Parse PPPNAVA
    // ------------------------------------------

    // ------------------------------------------
    // Actualizar yaw
    // ------------------------------------------

    readYaw(currentYaw);

    // ------------------------------------------
    // Máquina de estados
    // ------------------------------------------

    switch(posState)
    {
        case MOVING:
            break;

        case STOP_CONFIRM:
            break;

        case AVERAGING:
            break;

        case LOCKED:
            break;
    }

    // ------------------------------------------
    // Salida Dynatest
    // ------------------------------------------

    static uint32_t lastTx = 0;

    if(millis() - lastTx >= GGA_PERIOD_MS)
    {
        lastTx = millis();

        if(lockedValid)
        {
            String gga =
                buildGPGGA(
                    utcTime,
                    latLocked,
                    lonLocked,
                    altLocked,
                    outputFixQ(),
                    18);

            DYN.println(gga);
        }
    }
}
