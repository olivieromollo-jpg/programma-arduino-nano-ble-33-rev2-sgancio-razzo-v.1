// ============================================================
//  
//  Firmware razzo modello — Arduino Nano 33 BLE
//  
// ============================================================

// ── LIBRERIE ─────────────────────────────────────────────────
#include <ArduinoBLE.h>               // Gestione Bluetooth Low Energy integrato nel Nano 33 BLE
#include <Arduino_BMI270_BMM150.h>    // Driver per l'IMU (accelerometro/giroscopio BMI270 + magnetometro BMM150)
#include <Wire.h>                     // Protocollo I²C — necessario per comunicare col barometro LPS22DF
#include <Servo.h>                    // Libreria per controllare il servo che apre il paracadute

// ── PIN FISICI ────────────────────────────────────────────────
const int PIN_SERVO        = 2;       // Pin digitale D2 → cavo segnale del servo del paracadute
const int PIN_LED_RECUPERO = 4;       // Pin digitale D4 → LED che lampeggia in fase di recupero a terra
const int PIN_BUZZER       = 5;       // Pin digitale D5 → cicalino piezoelettrico per segnalazione sonora

// ── COSTANTI FISICHE ──────────────────────────────────────────
const float G  = 9.80665f;           // Accelerazione di gravità standard [m/s²]
const float DT = 0.01f;              // Passo di integrazione temporale [s] = 10 ms

// ── SOGLIE — MODALITÀ TEST A MANO ────────────────────────────
const float ALT_MINIMA               = 0.3f;
const float SOGLIA_BOOST             = 1.5f;
const float SOGLIA_FINE_BOOST        = 0.0f;
const float SOGLIA_VELOCITA_APOGEO   = 0.15f;
const float SOGLIA_CALO_ALTEZZA      = 0.05f;
const unsigned long DEBOUNCE_APOGEO_MS = 200;

// ── SOGLIE — VOLO REALE  ─
// const float ALT_MINIMA               = 1.67f;
// const float SOGLIA_BOOST             = 7.0f;
// const float SOGLIA_FINE_BOOST        = 0.0f;
// const float SOGLIA_VELOCITA_APOGEO   = 0.2f;
// const float SOGLIA_CALO_ALTEZZA      = 0.3f;
// const unsigned long DEBOUNCE_APOGEO_MS = 300;

// ── COSTANTI BAROMETRO ────────────────────────────────────────
const int LPS22DF_ADDR  = 0x5C;
const int BAR_FILTER_SIZE = 8;

// ── ENUM STATI DI VOLO ────────────────────────────────────────
enum FlightState {
  IDLE,    // 0 — Razzo a terra, in attesa del decollo
  BOOST,   // 1 — Motore acceso, accelerazione positiva rilevata
  COAST,   // 2 — Motore spento, razzo in salita per inerzia
  APOGEO,  // 3 — Punto più alto raggiunto, servo del paracadute attivato
  DISCESA  // 4 — Razzo in discesa, LED e buzzer attivi per il recupero
};

// ── OGGETTO SERVO ─────────────────────────────────────────────
Servo servoParacadute;

// ── VARIABILI DI STATO GLOBALE ────────────────────────────────
FlightState statoVolo = IDLE;
float velocity        = 0.0f;
float altezzaAttuale  = 0.0f;
float altezzaMassima  = 0.0f;
float pressioneBase   = 1013.25f;
float angoloVerticale = 0.0f;        // Angolo verticale stimato [°] — aggiornato dall'IMU
bool  isSystemArmed   = false;       // true = sistema armato e pronto al volo

// ── BUFFER BAROMETRO ──────────────────────────────────────────
float barBuffer[BAR_FILTER_SIZE] = {};
int   barIdx   = 0;
bool  barReady = false;

// ── TIMESTAMP E FLAG ─────────────────────────────────────────
unsigned long tsApogeoRilevato   = 0;
unsigned long tsApogeoCandidate  = 0;
unsigned long tsUltimoLog        = 0;
unsigned long tsUltimoSegnale    = 0;
unsigned long ultimoTempoIntegrazione = 0; // Timestamp [µs] dell'ultima integrazione — usato per DT preciso
bool segnaleOn       = false;
bool apogeoCandidate = false;

// Debounce decollo
bool boostCandidate = false;
unsigned long boostStart = 0;

// Debounce fine boost
bool coastCandidate = false;
unsigned long coastStart = 0;

// ── CARATTERISTICHE BLE ───────────────────────────────────────
BLEService rocketService("19B10000-E8F2-537E-4F6C-D104768A1214");

BLEFloatCharacteristic bleAltezza ("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEFloatCharacteristic bleVelocita("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEFloatCharacteristic bleAltMax  ("19B10003-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEByteCharacteristic  bleStato   ("19B10004-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEFloatCharacteristic bleAngolo  ("19B10005-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEByteCharacteristic  bleComando ("19B10006-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

// ============================================================
//  FUNZIONE HELPER: beep()
//  Emette un tono sul buzzer per la durata specificata.
// ============================================================
void beep(int freq, int durataMs) {
  tone(PIN_BUZZER, freq, durataMs);
  delay(durataMs);
}

// ============================================================
//  FUNZIONE: leggiPressioneRaw()
// ============================================================
float leggiPressioneRaw() {
  Wire1.beginTransmission(LPS22DF_ADDR);
  Wire1.write(0x28);
  if (Wire1.endTransmission() != 0)
    return 0.0f;

  Wire1.requestFrom(LPS22DF_ADDR, 3);
  if (Wire1.available() < 3)
    return 0.0f;

  uint32_t xl = Wire1.read();
  uint32_t l  = Wire1.read();
  uint32_t h  = Wire1.read();

  return (float)((h << 16) | (l << 8) | xl) / 4096.0f;
}

// ============================================================
//  FUNZIONE: leggiPressioneFiltrata()
// ============================================================
float leggiPressioneFiltrata() {
  float raw = leggiPressioneRaw();

  if (raw < 500.0f) return 0.0f;

  barBuffer[barIdx] = raw;
  barIdx = (barIdx + 1) % BAR_FILTER_SIZE;
  if (barIdx == 0) barReady = true;

  int   n   = barReady ? BAR_FILTER_SIZE : barIdx;
  float sum = 0.0f;
  for (int i = 0; i < n; i++) sum += barBuffer[i];
  return sum / (float)n;
}

// ============================================================
//  FUNZIONE: pressioneToAltezza()
// ============================================================
float pressioneToAltezza(float p) {
  if (p < 500.0f || pressioneBase < 500.0f) return 0.0f;
  return 44330.0f * (1.0f - powf(p / pressioneBase, 0.1903f));
}

// ============================================================
//  FUNZIONE: calibraBarometro()
// ============================================================
void calibraBarometro() {
  Serial.println("Calibrazione barometro...");
  float sum = 0.0f;
  int   n   = 0;

  for (int i = 0; i < 30; i++) {
    float p = leggiPressioneRaw();
    if (p > 500.0f) { sum += p; n++; }
    delay(50);
  }

  if (n > 0) {
    pressioneBase = sum / (float)n;
    Serial.print("Pressione base: ");
    Serial.print(pressioneBase);
    Serial.println(" hPa");
  } else {
    Serial.println("ATTENZIONE: barometro non risponde, uso 1013.25 hPa");
  }
}

// ============================================================
//  FUNZIONE: aggiornaStato()
// ============================================================
void aggiornaStato(float aVertical) {
  switch (statoVolo) {

    case IDLE:
      if (!isSystemArmed) break;  // Non parte se il sistema non è armato via BLE
      if (aVertical > SOGLIA_BOOST) {
        if (!boostCandidate) {
          boostCandidate = true;
          boostStart = millis();
        }
        else if (millis() - boostStart >= 100) {
          statoVolo = BOOST;
          velocity = 0.0f;
          Serial.println(">>> DECOLLO — stato: BOOST");
          boostCandidate = false;
        }
      } else {
        boostCandidate = false;
      }
      break;

    case BOOST:
      if (aVertical < SOGLIA_FINE_BOOST) {
        if (!coastCandidate) {
          coastCandidate = true;
          coastStart = millis();
        }
        else if (millis() - coastStart >= 100) {
          statoVolo = COAST;
          Serial.println(">>> Motore spento — stato: COAST");
          coastCandidate = false;
        }
      } else {
        coastCandidate = false;
      }
      break;

    case COAST: {
      bool velocitaBassa = (velocity <= SOGLIA_VELOCITA_APOGEO);
      bool quotaInCalo   = (altezzaAttuale < altezzaMassima - SOGLIA_CALO_ALTEZZA);
      bool quotaSicura   = (altezzaMassima >= ALT_MINIMA);

      if (velocitaBassa && quotaInCalo && quotaSicura) {
        if (!apogeoCandidate) {
          apogeoCandidate   = true;
          tsApogeoCandidate = millis();
        } else if (millis() - tsApogeoCandidate >= DEBOUNCE_APOGEO_MS) {
          statoVolo        = APOGEO;
          tsApogeoRilevato = millis();
          servoParacadute.write(90);
          tone(PIN_BUZZER, 2500, 300);
          Serial.println(">>> APOGEO — paracadute aperto");
        }
      } else {
        apogeoCandidate = false;
      }
      break;
    }

    case APOGEO:
      if (millis() - tsApogeoRilevato >= 1000) {
        statoVolo = DISCESA;
        Serial.println(">>> DISCESA — segnalazione attiva");
      }
      break;

    case DISCESA:
      break;
  }
}

// ============================================================
//  FUNZIONE: aggiornaSegnalazione()
// ============================================================
void aggiornaSegnalazione() {
  if (statoVolo != DISCESA) return;

  if (millis() - tsUltimoSegnale > 150) {
    tsUltimoSegnale = millis();
    segnaleOn = !segnaleOn;

    digitalWrite(PIN_LED_RECUPERO, segnaleOn ? HIGH : LOW);

    if (segnaleOn) tone(PIN_BUZZER, 1500);
    else           noTone(PIN_BUZZER);
  }
}

// ============================================================
//  FUNZIONE: aggiornaBLE()
//  Gestisce ricezione comandi e trasmissione telemetria.
// ============================================================
void aggiornaBLE() {
  BLEDevice central = BLE.central();

  // ── RICEZIONE COMANDI ────────────────────────────────────
  if (central && central.connected()) {
    if (bleComando.written()) {
      if (bleComando.value() == 1) {
        // Comando ARM: arma il sistema e azzera tutto
        isSystemArmed = true;
        statoVolo = IDLE;
        velocity = 0.0f;
        pressioneBase = leggiPressioneFiltrata();
        altezzaAttuale = 0.0f;
        altezzaMassima = 0.0f;
        for (int i = 0; i < 3; i++) { beep(2000, 100); delay(100); }
        tsUltimoSegnale = millis();
        ultimoTempoIntegrazione = micros();
        Serial.println(">>> SISTEMA ARMATO via BLE");
      } else {
        // Comando DISARM: disarma il sistema e mette tutto in sicurezza
        isSystemArmed = false;
        statoVolo = IDLE;
        velocity = 0.0f;
        servoParacadute.write(0);
        digitalWrite(PIN_LED_RECUPERO, LOW);
        beep(800, 500);
        Serial.println(">>> SISTEMA DISARMATO via BLE");
      }
    }
  }

  // ── TRASMISSIONE TELEMETRIA (ogni 100 ms, solo se connesso) ──
  if (central && central.connected()) {
    static unsigned long lastBleSend = 0;
    if (millis() - lastBleSend >= 100) {
      lastBleSend = millis();
      bleAltezza.writeValue(altezzaAttuale);
      bleVelocita.writeValue(velocity);
      bleAltMax.writeValue(altezzaMassima);
      bleStato.writeValue((byte)statoVolo);
      bleAngolo.writeValue(angoloVerticale);
    }
  }
}

// ============================================================
//  FUNZIONE: stampaLog()
// ============================================================
void stampaLog() {
  if (millis() - tsUltimoLog < 250) return;
  tsUltimoLog = millis();

  const char* nomiStato[] = {"IDLE", "BOOST", "COAST", "APOGEO", "DISCESA"};

  Serial.print(nomiStato[statoVolo]);
  Serial.print(isSystemArmed ? " [ARMATO]" : " [DISARMATO]");
  Serial.print(" | Alt: ");
  Serial.print(altezzaAttuale, 2);
  Serial.print(" m");
  Serial.print(" | Max: ");
  Serial.print(altezzaMassima, 2);
  Serial.print(" m");
  Serial.print(" | Vel: ");
  Serial.print(velocity, 2);
  Serial.println(" m/s");
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(PIN_ENABLE_SENSORS_3V3, OUTPUT);
  pinMode(PIN_ENABLE_I2C_PULLUP,  OUTPUT);
  digitalWrite(PIN_ENABLE_SENSORS_3V3, HIGH);
  digitalWrite(PIN_ENABLE_I2C_PULLUP,  HIGH);
  delay(100);

  pinMode(PIN_LED_RECUPERO, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_LED_RECUPERO, LOW);
  noTone(PIN_BUZZER);

  servoParacadute.attach(PIN_SERVO);
  servoParacadute.write(0);
  delay(500);

  Wire1.begin();
  if (!IMU.begin()) {
    Serial.println("ERRORE: IMU non trovata!");
    while (1);
  }

  Wire1.beginTransmission(LPS22DF_ADDR);
  Wire1.write(0x10);
  Wire1.write(0x50);
  Wire1.endTransmission();
  delay(100);

  calibraBarometro();

  // ── INIZIALIZZAZIONE BLE ─────────────────────────────────
  if (!BLE.begin()) {
    Serial.println("ERRORE: BLE non avviato!");
    while (1);
  }

  BLE.setLocalName("Nano33_Telemetry");
  BLE.setAdvertisedService(rocketService);

  rocketService.addCharacteristic(bleAltezza);
  rocketService.addCharacteristic(bleVelocita);
  rocketService.addCharacteristic(bleAltMax);
  rocketService.addCharacteristic(bleStato);
  rocketService.addCharacteristic(bleAngolo);
  rocketService.addCharacteristic(bleComando);

  BLE.addService(rocketService);

  bleAltezza.writeValue(0.0f);
  bleVelocita.writeValue(0.0f);
  bleAltMax.writeValue(0.0f);
  bleStato.writeValue((byte)IDLE);
  bleAngolo.writeValue(0.0f);
  bleComando.writeValue(0);

  BLE.advertise();

  ultimoTempoIntegrazione = micros();

  // Sequenza di avvio: 3 bip + lampeggi LED
  for (int i = 0; i < 3; i++) {
    digitalWrite(PIN_LED_RECUPERO, HIGH);
    tone(PIN_BUZZER, 2000, 100);
    delay(100);
    digitalWrite(PIN_LED_RECUPERO, LOW);
    delay(100);
  }

  Serial.println("=== MODALITA TEST A MANO ===");
  Serial.println("Sistema pronto. In attesa di ARM via BLE.");
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  if (!IMU.accelerationAvailable()) {
    delay(10);
    return;
  }

  float ax, ay, az;
  IMU.readAcceleration(ax, ay, az);

  float aVertical = (az * G) - G;

  if (statoVolo == BOOST || statoVolo == COAST) {
    velocity += aVertical * DT;
  } else if (statoVolo == IDLE) {
    velocity = 0.0f;
  }

  float pFilt = leggiPressioneFiltrata();
  if (pFilt > 0.0f) {
    altezzaAttuale = pressioneToAltezza(pFilt);
  }

  if ((statoVolo == BOOST || statoVolo == COAST) && altezzaAttuale > altezzaMassima) {
    altezzaMassima = altezzaAttuale;
  }

  aggiornaStato(aVertical);
  aggiornaSegnalazione();
  aggiornaBLE();
  stampaLog();

  delay(10);
}