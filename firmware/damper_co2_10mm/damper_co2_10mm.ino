/* =============================================================================
   FIRMWARE - DAMPER DE MICRO-VENTILACAO + CONTROLADOR PROPORCIONAL DE CO2
   Derivado de: jfgonjr/controlador-de-fluido-10mm (controlador de agua)
   -----------------------------------------------------------------------------
   MECANICA (INALTERADA - mesmo CAD):
     Corpo 88 x 50 x 62 mm | disco borboleta Ø32 | eixo Ø6 | AS5600 + ima diametral
     0 graus  = disco perpendicular ao fluxo = FECHADO (retem umidade/gases)
     90 graus = disco alinhado ao fluxo      = ABERTO  (exaustao maxima)
     Sem fins de curso: a posicao vem do AS5600 (malha fechada).
     3 passagens Ø7 na caixa: sensores ambientais + cabo do micro-exaustor.

   LOGICA DE CONTROLE (modo AUTO)
     CO2 <  Limite Inferior            -> 0 graus,  exaustor OFF
     Inferior <= CO2 < Superior        -> angulo proporcional ao excesso de CO2,
                                          quantizado em degraus (padrao 15 graus:
                                          15/30/45/60/75), com histerese em ppm
     CO2 >= Superior  OU  Umid critica -> 90 graus + exaustor 100 %

   SEGURANCA BIOLOGICA
     - Homing na partida: vai ao 0 pelo AS5600 antes de qualquer outra coisa.
     - Fail-safe: CO2 sem resposta > 10 s -> valvula a 45 graus + alerta serial.
     - Modo manual expira sozinho (evita esquecer a valvula forcada).
     - Setpoints, histerese, zero e sentido do encoder ficam em EEPROM/NVS.

   BIBLIOTECAS (Gerenciador de Bibliotecas da IDE) - so as da opcao escolhida:
     TH_DHT22  : "DHT sensor library" (Adafruit) + "Adafruit Unified Sensor"
     TH_BME280 : "Adafruit BME280 Library" + "Adafruit Unified Sensor"
     CO2_SCD30 : "SparkFun SCD30 Arduino Library"
     CO2_MHZ19B: nenhuma (protocolo implementado aqui)
   ESP32: requer Arduino-ESP32 core >= 3.0 (analogWrite/tone nativos).

   COMANDOS SERIAIS (115200 baud, terminar com Enter):
     S                       mostra a configuracao atual
     S<inf>,<sup>[,<hist>[,<umidCrit>[,<passo>]]]
                             ex: S600,1200,50,95,15   (grava na EEPROM/NVS)
     Z                       grava a posicao ATUAL como 0 graus (valvula FECHADA!)
     I                       inverte o sentido de leitura do encoder
     M<graus>                modo manual, ex: M45 (0..90). Expira em 10 min.
     F<0-100>                (so em manual) exaustor em %, ex: F60
     A                       volta ao controle automatico
     H                       refaz o homing (vai ao 0)
     ?                       status
   ============================================================================= */

#include <Wire.h>
#include <math.h>

/* =============================================================================
   1) SELECAO DE HARDWARE
   ============================================================================= */
#define CO2_MHZ19B 1          // UART (9600 baud)
#define CO2_SCD30  2          // I2C 0x61 (tambem fornece temperatura/umidade)
#define CO2_SENSOR CO2_MHZ19B // <<< selecione aqui

#define TH_DHT22   1
#define TH_BME280  2          // I2C 0x76/0x77
#define TH_SCD30   3          // usa o proprio SCD30 (exige CO2_SCD30)
#define TH_SENSOR  TH_DHT22   // <<< selecione aqui

#if (TH_SENSOR == TH_SCD30) && (CO2_SENSOR != CO2_SCD30)
  #error "TH_SCD30 exige CO2_SENSOR == CO2_SCD30"
#endif

#if defined(ESP32)
  #include <Preferences.h>
  Preferences prefs;
#else
  #include <EEPROM.h>
  #include <SoftwareSerial.h>
#endif

#if CO2_SENSOR == CO2_SCD30
  #include <SparkFun_SCD30_Arduino_Library.h>
  SCD30 scd30;
#endif

#if TH_SENSOR == TH_DHT22
  #include <DHT.h>
#elif TH_SENSOR == TH_BME280
  #include <Adafruit_BME280.h>
  Adafruit_BME280 bme;
#endif

/* =============================================================================
   2) PINAGEM (mesma base do projeto original; ajuste a sua placa de 54 x 34 mm)
   -----------------------------------------------------------------------------
   Removidos: sonda de vazamento, alagamento, fluxo e botoes (agora sem uso).
   AVR: tone() usa o Timer2 e desativa o PWM dos pinos 3 e 11. Por isso o
   exaustor ficou no pino 10 (Timer1) e o LED verde foi para o pino 3.
   ============================================================================= */
#if defined(ESP32)
const uint8_t PIN_SDA       = 8;
const uint8_t PIN_SCL       = 9;
const uint8_t PIN_MOT_A     = 3;   // ponte H IN1 (PWM)
const uint8_t PIN_MOT_B     = 4;   // ponte H IN2 (PWM)
const uint8_t PIN_EXAUSTOR  = 5;   // PWM -> MOSFET do micro-exaustor 12 V
const uint8_t PIN_CO2_RX    = 6;   // RX do ESP32  <- TX do MH-Z19B
const uint8_t PIN_CO2_TX    = 7;   // TX do ESP32  -> RX do MH-Z19B
const uint8_t PIN_DHT       = 10;
const uint8_t PIN_BUZZER    = 2;
const uint8_t PIN_LED_VERDE = 20;
const uint8_t PIN_LED_VERM  = 21;
#else // Arduino Nano / Pro Mini (ATmega328P)
const uint8_t PIN_MOT_A     = 5;   // PWM (Timer0)
const uint8_t PIN_MOT_B     = 6;   // PWM (Timer0)
const uint8_t PIN_EXAUSTOR  = 10;  // PWM (Timer1)
const uint8_t PIN_CO2_RX    = 2;   // RX do Arduino <- TX do MH-Z19B
const uint8_t PIN_CO2_TX    = 4;   // TX do Arduino -> RX do MH-Z19B
const uint8_t PIN_DHT       = 7;
const uint8_t PIN_BUZZER    = 9;
const uint8_t PIN_LED_VERDE = 3;
const uint8_t PIN_LED_VERM  = 11;
// I2C: A4 (SDA) / A5 (SCL)
#endif

/* =============================================================================
   3) PARAMETROS (mecanismo, controle e tempos)
   ============================================================================= */
// --- Mecanismo (do CAD / firmware original) ---
const float ANG_FECHADO   = 0.0f;
const float ANG_ABERTO    = 90.0f;
const float ANG_FAILSAFE  = 45.0f;
const float TOL_PARADA    = 2.5f;   // graus: motor para ao chegar nesta janela
const float ENGATE_EXTREMO = 3.5f;  // graus: reengata se sair disso em 0/90
const float ENGATE_INTERM  = 4.5f;  // graus: reengata se sair disso em angulos intermediarios
const float ANG_DESACEL   = 20.0f;  // inicio da rampa de frenagem
const uint8_t PWM_MIN     = 90;     // vence o atrito da bucha Ø6
const uint8_t PWM_MAX     = 255;

// --- Padroes de controle (sobrescritos pela EEPROM/NVS apos o comando 'S') ---
const uint16_t PADRAO_CO2_INF  = 600;   // ppm
const uint16_t PADRAO_CO2_SUP  = 1200;  // ppm
const uint16_t PADRAO_CO2_HIST = 50;    // ppm de histerese para FECHAR/REDUZIR
const uint8_t  PADRAO_UMID_CRIT = 95;   // %UR que forca abertura total
const uint8_t  PADRAO_UMID_HIST = 3;    // %UR de histerese da umidade critica
const uint8_t  PADRAO_PASSO     = 15;   // graus por degrau na faixa proporcional

// --- Exaustor ---
const uint8_t FAN_PWM_MIN        = 90;     // minimo para a ventoinha partir
const float   ANG_FAN_LIBERA     = 75.0f;  // so liga o exaustor com a valvula quase aberta
const bool    FAN_FAIXA_MEDIA    = false;  // true: ventoinha proporcional tambem na faixa media
const uint8_t FAILSAFE_FAN_PWM   = 0;      // 0..255 durante o fail-safe (0 = desligado)

// --- Tempos (ms) ---
const unsigned long T_FAILSAFE_CO2   = 10000UL;   // sem leitura valida -> fail-safe
const unsigned long T_WARMUP_CO2     = 60000UL;   // MH-Z19B: datasheet pede ~3 min (180000UL)
const unsigned long T_CO2_POLL       = 2000UL;
const unsigned long T_TH_POLL        = 3000UL;    // DHT22: minimo 2 s
const unsigned long T_TH_STALE       = 30000UL;
const unsigned long T_TELEMETRIA     = 1000UL;
const unsigned long T_MANOBRA_MAX    = 8000UL;
const unsigned long T_STALL          = 1200UL;
const unsigned long T_MANUAL_MAX     = 600000UL;  // 10 min
const unsigned long T_RETRY_FALHA    = 30000UL;
const uint8_t       CO2_LEITURAS_RECUPERA = 3;    // leituras validas para sair do fail-safe
const bool          BUZZER_ATIVO     = true;      // bipe de alerta (fail-safe/falhas)

/* =============================================================================
   4) CONFIGURACAO PERSISTENTE (EEPROM no AVR, NVS/Preferences no ESP32)
   ============================================================================= */
struct Config {
  uint16_t magic;
  uint16_t co2Inf, co2Sup, co2Hist;
  uint8_t  umidCrit, umidHist, passo;
  uint8_t  zeroCalibrado;
  uint16_t zeroBruto;          // leitura bruta do AS5600 na posicao FECHADA
  int8_t   sentido;            // +1 / -1
};
const uint16_t MAGIC = 0xC0B2;
Config cfg;

#if !defined(ESP32)
// Layout do firmware de agua (para aproveitar a calibracao ja feita)
struct ConfigLegado { uint16_t magic; uint16_t zeroBruto; int8_t sentido; uint8_t bloqueado; };
const uint16_t MAGIC_LEGADO = 0xC0A1;
#endif

void cfgPadraoSetpoints() {
  cfg.co2Inf = PADRAO_CO2_INF;  cfg.co2Sup = PADRAO_CO2_SUP;
  cfg.co2Hist = PADRAO_CO2_HIST;
  cfg.umidCrit = PADRAO_UMID_CRIT; cfg.umidHist = PADRAO_UMID_HIST;
  cfg.passo = PADRAO_PASSO;
}
void cfgPadraoTudo() {
  cfg.magic = MAGIC;
  cfgPadraoSetpoints();
  cfg.zeroCalibrado = 0; cfg.zeroBruto = 0; cfg.sentido = 1;
}
bool setpointsValidos() {
  return cfg.co2Inf >= 200 && cfg.co2Inf < cfg.co2Sup && cfg.co2Sup <= 10000 &&
         cfg.co2Hist >= 5 && cfg.co2Hist <= (cfg.co2Sup - cfg.co2Inf) / 2 &&
         cfg.umidCrit >= 50 && cfg.umidCrit <= 100 && cfg.umidHist <= 30 &&
         cfg.passo >= 5 && cfg.passo <= 45;
}
void salvarCfg() {
#if defined(ESP32)
  prefs.putBytes("cfg", &cfg, sizeof(cfg));
#else
  EEPROM.put(0, cfg);              // put() so grava os bytes que mudaram
#endif
}
void carregarCfg() {
  bool lido = true;
#if defined(ESP32)
  prefs.begin("damper", false);
  lido = (prefs.getBytes("cfg", &cfg, sizeof(cfg)) == sizeof(cfg));
#else
  EEPROM.get(0, cfg);
  if (cfg.magic == MAGIC_LEGADO) {                 // migra zero/sentido do firmware antigo
    ConfigLegado leg; EEPROM.get(0, leg);
    cfgPadraoTudo();
    cfg.zeroBruto = leg.zeroBruto; cfg.sentido = (leg.sentido < 0) ? -1 : 1;
    cfg.zeroCalibrado = 1;
    salvarCfg();
    Serial.println(F("{\"INFO\":\"ZERO_MIGRADO_DO_FIRMWARE_AGUA\",\"acao\":\"CONFIRME_COM_VALVULA_FECHADA_OU_USE_Z\"}"));
    return;
  }
#endif
  if (!lido || cfg.magic != MAGIC) { cfgPadraoTudo(); salvarCfg(); return; }
  if (!setpointsValidos()) { cfgPadraoSetpoints(); salvarCfg(); }
  if (cfg.sentido != 1 && cfg.sentido != -1) { cfg.sentido = 1; salvarCfg(); }
}

/* =============================================================================
   5) ENCODER MAGNETICO AS5600 (I2C 0x36)
   ============================================================================= */
const uint8_t AS5600_ADDR   = 0x36;
const uint8_t AS5600_RAWANG = 0x0C;   // 12 bits, 0..4095
const uint8_t AS5600_STATUS = 0x0B;   // bit MD (0x20) = ima detectado

bool as5600Ler(uint16_t &bruto) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_RAWANG);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2) != 2) return false;
  uint16_t hi = Wire.read(), lo = Wire.read();
  bruto = ((hi << 8) | lo) & 0x0FFF;
  return true;
}
bool as5600ImaOk() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_STATUS);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)1) != 1) return false;
  return (Wire.read() & 0x20) != 0;
}
bool lerAnguloEncoder(float &ang) {
  uint16_t bruto;
  if (!as5600Ler(bruto)) return false;
  int16_t delta = (int16_t)bruto - (int16_t)cfg.zeroBruto;
  if (delta > 2048)  delta -= 4096;
  if (delta < -2048) delta += 4096;
  ang = cfg.sentido * delta * (360.0f / 4096.0f);
  return true;
}

/* =============================================================================
   6) ESTADO GLOBAL
   ============================================================================= */
enum Modo : uint8_t { MODO_SEM_ZERO, MODO_HOMING, MODO_AUTO, MODO_MANUAL };
Modo modo = MODO_SEM_ZERO;

unsigned long agora = 0, tBoot = 0;

// Posicionador
float anguloAtual = 0.0f;
float alvo = ANG_FECHADO;            // alvo corrente do posicionador
bool  encoderOk = false;
unsigned long tEncoderOk = 0;
bool  movendo = false, forcarManobra = false, falhaMec = false;
uint8_t stalls = 0;
unsigned long tManobra = 0, tRefStall = 0, tFalhaMec = 0;
float anguloRefStall = 0.0f;

// Sensores
uint16_t co2ppm = 0;   bool co2Tem = false;
unsigned long tCo2Ok = 0;
uint8_t co2Validos = 0;
float temperatura = NAN, umidade = NAN;
unsigned long tThOk = 0;
bool failsafe = false;

// Controle
float ultimoAlvoAuto = 0.0f;
bool  umidLatch = false;
float alvoManual = 0.0f;
unsigned long tManual = 0;
uint8_t fanManualPct = 0;
uint8_t fanPwm = 0;

// Throttle de alertas
unsigned long tAlCo2 = 0, tAlTh = 0, tAlEnc = 0, tAlMec = 0, tAlZero = 0, tAlIma = 0, tTelem = 0;

bool aCada(unsigned long &ref, unsigned long periodo) {
  if (agora - ref >= periodo) { ref = agora; return true; }
  return false;
}

/* =============================================================================
   7) PWM / MOTOR (ponte H DRV8833 ou L9110) E EXAUSTOR
   ============================================================================= */
void pwmIniciar() {
#if defined(ESP32)
  analogWriteFrequency(PIN_MOT_A, 20000);    // acima do audivel
  analogWriteFrequency(PIN_MOT_B, 20000);
  analogWriteFrequency(PIN_EXAUSTOR, 20000);
#endif
}
void motorParar() { analogWrite(PIN_MOT_A, 0); analogWrite(PIN_MOT_B, 0); }

/* pwm > 0 abre (angulo crescente), pwm < 0 fecha */
void motorAcionar(int pwm) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  if (pwm > 0)      { analogWrite(PIN_MOT_B, 0); analogWrite(PIN_MOT_A, pwm); }
  else if (pwm < 0) { analogWrite(PIN_MOT_A, 0); analogWrite(PIN_MOT_B, -pwm); }
  else motorParar();
}
void exaustorAplicar(uint8_t pwm) {
  fanPwm = pwm;
  analogWrite(PIN_EXAUSTOR, pwm);
}

/* =============================================================================
   8) SERIAL: ALERTAS JSON
   ============================================================================= */
void alerta(const __FlashStringHelper *codigo, const __FlashStringHelper *acao = nullptr) {
  Serial.print(F("{\"ALERTA\":\""));
  Serial.print(codigo);
  Serial.print('"');
  if (acao) { Serial.print(F(",\"acao\":\"")); Serial.print(acao); Serial.print('"'); }
  Serial.println('}');
}

/* =============================================================================
   9) SENSOR DE CO2
   ============================================================================= */
#if CO2_SENSOR == CO2_MHZ19B
#if defined(ESP32)
  HardwareSerial &co2Serial = Serial1;
#else
  SoftwareSerial co2Serial(PIN_CO2_RX, PIN_CO2_TX);
#endif
const uint8_t MHZ_CMD_LER[9]    = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
const uint8_t MHZ_CMD_ABC_OFF[9] = {0xFF, 0x01, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x86};
const bool MHZ_DESATIVAR_ABC = true;  // ABC assume 400 ppm minimo/24 h: errado em terrario fechado
uint8_t mhzBuf[9], mhzIdx = 0;
bool mhzEsperando = false;
unsigned long tMhzReq = 0, tCo2Poll = 0;
#endif

bool thOcupado() {   // evita ler o DHT22 (interrupcoes desligadas) no meio de um quadro UART
#if CO2_SENSOR == CO2_MHZ19B
  return mhzEsperando;
#else
  return false;
#endif
}

void co2Iniciar() {
#if CO2_SENSOR == CO2_MHZ19B
  #if defined(ESP32)
    co2Serial.begin(9600, SERIAL_8N1, PIN_CO2_RX, PIN_CO2_TX);
  #else
    co2Serial.begin(9600);
  #endif
  if (MHZ_DESATIVAR_ABC) { co2Serial.write(MHZ_CMD_ABC_OFF, 9); delay(50); }
#elif CO2_SENSOR == CO2_SCD30
  if (!scd30.begin(Wire, false)) alerta(F("SCD30_NAO_ENCONTRADO"));
  scd30.setMeasurementInterval(2);
  scd30.setAutoSelfCalibration(false);
#endif
}

void co2RegistrarLeitura(uint16_t ppm) {
  if (ppm < 100 || ppm > 10000) return;    // implausivel: nao conta como "vivo"
  co2ppm = ppm; co2Tem = true; tCo2Ok = agora;
  if (co2Validos < 255) co2Validos++;
}

void co2Atualizar() {
#if CO2_SENSOR == CO2_MHZ19B
  if (!mhzEsperando) {
    if (agora - tCo2Poll >= T_CO2_POLL) {
      while (co2Serial.available()) co2Serial.read();
      co2Serial.write(MHZ_CMD_LER, 9);
      mhzEsperando = true; mhzIdx = 0; tMhzReq = agora; tCo2Poll = agora;
    }
    return;
  }
  while (co2Serial.available()) {
    uint8_t b = co2Serial.read();
    if (mhzIdx == 0 && b != 0xFF) continue;     // ressincroniza no byte inicial
    mhzBuf[mhzIdx++] = b;
    if (mhzIdx == 9) {
      uint8_t soma = 0;
      for (uint8_t i = 1; i < 8; i++) soma += mhzBuf[i];
      uint8_t chk = (uint8_t)(0xFF - soma + 1);
      if (mhzBuf[1] == 0x86 && mhzBuf[8] == chk)
        co2RegistrarLeitura(((uint16_t)mhzBuf[2] << 8) | mhzBuf[3]);
      mhzEsperando = false;
      break;
    }
  }
  if (mhzEsperando && (agora - tMhzReq > 500)) mhzEsperando = false;  // timeout do quadro
#elif CO2_SENSOR == CO2_SCD30
  if (scd30.dataAvailable()) {
    co2RegistrarLeitura((uint16_t)scd30.getCO2());
  #if TH_SENSOR == TH_SCD30
    temperatura = scd30.getTemperature(); umidade = scd30.getHumidity(); tThOk = agora;
  #endif
  }
#endif
}

/* =============================================================================
   10) SENSOR DE TEMPERATURA / UMIDADE
   ============================================================================= */
#if TH_SENSOR == TH_DHT22
DHT dht(PIN_DHT, DHT22);
#endif
unsigned long tThPoll = 0;

void thIniciar() {
#if TH_SENSOR == TH_DHT22
  dht.begin();
#elif TH_SENSOR == TH_BME280
  if (!bme.begin(0x76) && !bme.begin(0x77)) alerta(F("BME280_NAO_ENCONTRADO"));
#endif
}
void thAtualizar() {
#if TH_SENSOR == TH_DHT22 || TH_SENSOR == TH_BME280
  if (agora - tThPoll < T_TH_POLL || thOcupado()) return;
  tThPoll = agora;
  float t, h;
  #if TH_SENSOR == TH_DHT22
    h = dht.readHumidity(); t = dht.readTemperature();
  #else
    h = bme.readHumidity(); t = bme.readTemperature();
  #endif
  if (!isnan(t) && !isnan(h) && h >= 0 && h <= 100 && t > -40 && t < 80) {
    temperatura = t; umidade = h; tThOk = agora;
  }
#endif
  if (!isnan(umidade) && (agora - tThOk > T_TH_STALE)) {   // dado velho nao pode comandar nada
    umidade = NAN; temperatura = NAN;
  }
}
bool thValido() { return !isnan(umidade); }

/* =============================================================================
   11) POSICIONADOR (malha fechada pelo AS5600)
   ============================================================================= */
void setAlvo(float a) { alvo = constrain(a, ANG_FECHADO, ANG_ABERTO); }

void atualizarAngulo() {
  float a;
  if (lerAnguloEncoder(a)) { anguloAtual = a; encoderOk = true; tEncoderOk = agora; }
  else if (encoderOk && agora - tEncoderOk > 500) {          // sem encoder = sem malha fechada
    encoderOk = false; motorParar(); movendo = false;
  }
}

/* Perfil de velocidade: PWM cheio longe do alvo, rampa nos ultimos graus. */
void irParaAlvo(float erro) {
  float mod = fabs(erro);
  float k = (mod >= ANG_DESACEL) ? 1.0f : (mod / ANG_DESACEL);
  int pwm = PWM_MIN + (int)((PWM_MAX - PWM_MIN) * k);
  motorAcionar(erro > 0 ? pwm : -pwm);
}

void atuadorLoop() {
  if (modo == MODO_SEM_ZERO || !encoderOk || falhaMec) { motorParar(); movendo = false; return; }

  float erro = alvo - anguloAtual;
  bool extremo = (alvo <= ANG_FECHADO + 0.1f) || (alvo >= ANG_ABERTO - 0.1f);

  if (!movendo) {
    float engate = extremo ? ENGATE_EXTREMO : ENGATE_INTERM;
    if (!forcarManobra && fabs(erro) <= engate) return;      // em repouso: mantem
    movendo = true; tManobra = agora; anguloRefStall = anguloAtual; tRefStall = agora;
  }
  forcarManobra = false;

  if (fabs(erro) <= TOL_PARADA) { motorParar(); movendo = false; stalls = 0; return; }

  irParaAlvo(erro);

  // Travamento: motor acionado e angulo parado (o encoder faz o papel de sensor de corrente)
  if (fabs(anguloAtual - anguloRefStall) > 1.0f) { anguloRefStall = anguloAtual; tRefStall = agora; }
  if ((agora - tRefStall >= T_STALL) || (agora - tManobra > T_MANOBRA_MAX)) {
    motorParar(); movendo = false;
    if (++stalls <= 2) {
      alerta(F("TRAVAMENTO"), F("TENTANDO_DESENCRAVAR"));
      motorAcionar(erro > 0 ? -PWM_MAX : PWM_MAX);            // recua brevemente e insiste
      delay(200); motorParar(); delay(50);
    } else {
      falhaMec = true; tFalhaMec = agora;
      alerta(F("FALHA_MECANICA"), F("MOTOR_PARADO_NOVA_TENTATIVA_EM_30S"));
    }
  }
}

/* =============================================================================
   12) CONTROLE PROPORCIONAL DE CO2
   ============================================================================= */
/* Angulo desejado para um dado CO2, sem histerese:
   < inf -> 0 | >= sup -> 90 | no meio -> degraus de 'passo' proporcionais ao excesso. */
float anguloParaCo2(float ppm) {
  if (ppm < cfg.co2Inf)  return ANG_FECHADO;
  if (ppm >= cfg.co2Sup) return ANG_ABERTO;
  float frac  = (ppm - cfg.co2Inf) / (float)(cfg.co2Sup - cfg.co2Inf);   // 0..1
  float ang   = ceilf(frac * ANG_ABERTO / cfg.passo) * cfg.passo;
  float maxIn = ANG_ABERTO - cfg.passo;       // 90 so e atingido ao cruzar o limite superior
  if (ang < cfg.passo) ang = cfg.passo;
  if (ang > maxIn)     ang = maxIn;
  return ang;
}

/* Umidade critica com histerese (liga em >= crit, desliga abaixo de crit - hist). */
bool umidadeCritica() {
  if (!thValido()) { umidLatch = false; return false; }
  if (umidade >= cfg.umidCrit) umidLatch = true;
  else if (umidade < (float)cfg.umidCrit - cfg.umidHist) umidLatch = false;
  return umidLatch;
}

/* Subir e imediato; reduzir exige o CO2 cair 'co2Hist' ppm abaixo da fronteira do degrau. */
float calcularAlvoAuto() {
  float desejado = anguloParaCo2((float)co2ppm);
  if (desejado < ultimoAlvoAuto) {
    float seguro = anguloParaCo2((float)co2ppm + cfg.co2Hist);
    desejado = (seguro < ultimoAlvoAuto) ? seguro : ultimoAlvoAuto;
  }
  ultimoAlvoAuto = desejado;
  return desejado;
}

uint8_t calcularExaustor() {
  if (modo == MODO_MANUAL) return (uint8_t)((uint16_t)fanManualPct * 255 / 100);
  if (modo != MODO_AUTO || falhaMec || !encoderOk) return 0;
  if (alvo >= ANG_ABERTO - 0.5f)                       // faixa critica: 100 %
    return (anguloAtual >= ANG_FAN_LIBERA) ? 255 : 0;  // espera a valvula abrir
  if (failsafe) return FAILSAFE_FAN_PWM;
  if (FAN_FAIXA_MEDIA && alvo >= cfg.passo && anguloAtual >= alvo - ENGATE_INTERM) {
    float f = (alvo - cfg.passo) / (ANG_ABERTO - 2.0f * cfg.passo + 0.001f);
    return (uint8_t)(FAN_PWM_MIN + constrain(f, 0.0f, 1.0f) * (255 - FAN_PWM_MIN));
  }
  return 0;
}

void iniciarHoming() {
  modo = MODO_HOMING; setAlvo(ANG_FECHADO);
  forcarManobra = true; stalls = 0; falhaMec = false;
}

void atualizarControle() {
  switch (modo) {
    case MODO_SEM_ZERO: break;

    case MODO_HOMING:
      setAlvo(ANG_FECHADO);
      if (encoderOk && !movendo && !forcarManobra && !falhaMec && fabs(anguloAtual) <= ENGATE_EXTREMO) {
        modo = MODO_AUTO; ultimoAlvoAuto = 0;
        Serial.println(F("{\"INFO\":\"HOMING_OK\",\"Angulo\":0}"));
      }
      break;

    case MODO_MANUAL:
      setAlvo(alvoManual);
      break;

    case MODO_AUTO: {
      bool umidCrit = umidadeCritica();
      bool aquecendo = (agora - tBoot) < T_WARMUP_CO2;
      float a;
      if (umidCrit)          a = ANG_ABERTO;                // umidade critica vale em qualquer situacao
      else if (failsafe)     a = ANG_FAILSAFE;
      else if (aquecendo || !co2Tem) a = ANG_FECHADO;       // sensor aquecendo: mantem vedado
      else                   a = calcularAlvoAuto();
      if (umidCrit) ultimoAlvoAuto = ANG_ABERTO;            // saida da umidade respeita a histerese de CO2
      setAlvo(a);
      break;
    }
  }
  exaustorAplicar(calcularExaustor());
}

/* =============================================================================
   13) SUPERVISAO: FAIL-SAFE, EXPIRACAO DO MANUAL, REARME DE FALHAS
   ============================================================================= */
void supervisionar() {
  // --- Fail-safe do sensor de CO2 (>10 s sem leitura valida) ---
  if (!failsafe && (agora - tCo2Ok >= T_FAILSAFE_CO2)) {
    failsafe = true; co2Validos = 0; co2Tem = false;
    alerta(F("CO2_SEM_RESPOSTA"), F("VALVULA_45_GRAUS"));
    tAlCo2 = agora;
  } else if (failsafe && co2Validos >= CO2_LEITURAS_RECUPERA) {
    failsafe = false;
    Serial.println(F("{\"INFO\":\"CO2_RECUPERADO\",\"acao\":\"CONTROLE_AUTOMATICO\"}"));
  }
  if (failsafe && aCada(tAlCo2, 5000UL))
    alerta(F("CO2_SEM_RESPOSTA"), (modo == MODO_AUTO) ? F("VALVULA_45_GRAUS") : F("MANUAL_ATIVO_SEM_PROTECAO"));

  // --- Alertas das demais falhas (com limite de repeticao) ---
  if (modo != MODO_SEM_ZERO && !encoderOk && aCada(tAlEnc, 5000UL))
    alerta(F("ENCODER_AS5600_SEM_RESPOSTA"), F("MOTOR_PARADO"));
  if (modo == MODO_SEM_ZERO && aCada(tAlZero, 10000UL))
    alerta(F("ZERO_NAO_CALIBRADO"), F("FECHE_O_DISCO_E_ENVIE_Z"));
  if (!thValido() && aCada(tAlTh, 30000UL))
    alerta(F("TEMP_UMID_SEM_LEITURA"), F("UMIDADE_CRITICA_DESATIVADA"));
  if (encoderOk && aCada(tAlIma, 15000UL) && !as5600ImaOk())
    alerta(F("IMA_NAO_DETECTADO_NO_AS5600"));

  // --- Falha mecanica: nova tentativa periodica ---
  if (falhaMec) {
    if (aCada(tAlMec, 5000UL)) alerta(F("FALHA_MECANICA_ATIVA"));
    if (agora - tFalhaMec >= T_RETRY_FALHA) {
      falhaMec = false; stalls = 0;
      if (modo == MODO_HOMING) forcarManobra = true;
    }
  }

  // --- Modo manual expira sozinho ---
  if (modo == MODO_MANUAL && (agora - tManual >= T_MANUAL_MAX)) {
    modo = MODO_AUTO; fanManualPct = 0;
    Serial.println(F("{\"INFO\":\"MANUAL_EXPIRADO\",\"acao\":\"CONTROLE_AUTOMATICO\"}"));
  }
}

/* =============================================================================
   14) TELEMETRIA E COMANDOS SERIAIS
   ============================================================================= */
const __FlashStringHelper *nomeModo() {
  if (modo == MODO_SEM_ZERO) return F("SEM_ZERO");
  if (!encoderOk)            return F("ENCODER_FALHA");
  if (falhaMec)              return F("FALHA_MEC");
  if (modo == MODO_HOMING)   return F("HOMING");
  if (modo == MODO_MANUAL)   return F("MANUAL");
  if (failsafe)              return F("FAILSAFE");
  return F("AUTO");
}

void imprimirTelemetria() {
  Serial.print(F("{\"CO2_ppm\":"));
  if (co2Tem) Serial.print(co2ppm); else Serial.print(F("null"));
  Serial.print(F(",\"Temp\":"));
  if (thValido()) Serial.print(temperatura, 1); else Serial.print(F("null"));
  Serial.print(F(",\"Umid\":"));
  if (thValido()) Serial.print(umidade, 0); else Serial.print(F("null"));
  Serial.print(F(",\"Angulo\":"));
  if (encoderOk) Serial.print((int)lroundf(anguloAtual)); else Serial.print(F("null"));
  Serial.print(F(",\"Exaustor\":"));    Serial.print(fanPwm > 0 ? 1 : 0);
  Serial.print(F(",\"ExaustorPct\":")); Serial.print((int)(fanPwm * 100UL / 255));
  Serial.print(F(",\"Alvo\":"));        Serial.print((int)lroundf(alvo));
  Serial.print(F(",\"Modo\":\""));      Serial.print(nomeModo());
  Serial.println(F("\"}"));
}

void imprimirConfig() {
  Serial.print(F("{\"CFG\":{\"CO2_inf\":")); Serial.print(cfg.co2Inf);
  Serial.print(F(",\"CO2_sup\":"));          Serial.print(cfg.co2Sup);
  Serial.print(F(",\"Hist_ppm\":"));         Serial.print(cfg.co2Hist);
  Serial.print(F(",\"Umid_crit\":"));        Serial.print(cfg.umidCrit);
  Serial.print(F(",\"Umid_hist\":"));        Serial.print(cfg.umidHist);
  Serial.print(F(",\"Passo_graus\":"));      Serial.print(cfg.passo);
  Serial.print(F(",\"Zero_bruto\":"));       Serial.print(cfg.zeroBruto);
  Serial.print(F(",\"Sentido\":"));          Serial.print(cfg.sentido);
  Serial.println(F("}}"));
}

/* Extrai ate 'max' inteiros de uma string, separados por qualquer caractere nao numerico. */
uint8_t lerInteiros(const char *s, long *saida, uint8_t max) {
  uint8_t n = 0;
  while (*s && n < max) {
    while (*s && *s != '-' && (*s < '0' || *s > '9')) s++;
    if (!*s) break;
    char *fim;
    saida[n++] = strtol(s, &fim, 10);
    if (fim == s) break;
    s = fim;
  }
  return n;
}

void comandoSetpoints(const char *arg) {
  long v[5];
  uint8_t n = lerInteiros(arg, v, 5);
  if (n == 0) { imprimirConfig(); return; }
  if (n < 2)  { alerta(F("CMD_S_USO"), F("S<inf>,<sup>[,<hist>[,<umid>[,<passo>]]]")); return; }

  Config novo = cfg;
  novo.co2Inf = (uint16_t)constrain(v[0], 0L, 65535L);
  novo.co2Sup = (uint16_t)constrain(v[1], 0L, 65535L);
  if (n >= 3) novo.co2Hist  = (uint16_t)constrain(v[2], 0L, 65535L);
  if (n >= 4) novo.umidCrit = (uint8_t)constrain(v[3], 0L, 255L);
  if (n >= 5) novo.passo    = (uint8_t)constrain(v[4], 0L, 255L);

  Config anterior = cfg;
  cfg = novo;
  if (!setpointsValidos()) {
    cfg = anterior;
    alerta(F("SETPOINTS_INVALIDOS"), F("REGRAS_200<=inf<sup<=10000_hist5..(sup-inf)/2_umid50..100_passo5..45"));
    return;
  }
  salvarCfg();
  ultimoAlvoAuto = 0;
  Serial.println(F("{\"OK\":\"SETPOINTS_SALVOS\"}"));
  imprimirConfig();
}

void processarComando(char *s) {
  while (*s == ' ') s++;
  if (!*s) return;
  char c = toupper(*s);
  const char *arg = s + 1;
  long v[1];

  switch (c) {
    case 'S': comandoSetpoints(arg); break;

    case 'Z': {
      uint16_t bruto;
      if (!as5600Ler(bruto)) { alerta(F("ENCODER_AS5600_SEM_RESPOSTA")); break; }
      cfg.zeroBruto = bruto; cfg.zeroCalibrado = 1; salvarCfg();
      Serial.println(F("{\"OK\":\"ZERO_GRAVADO\",\"nota\":\"disco deve estar FECHADO\"}"));
      iniciarHoming();
      break;
    }
    case 'I':
      cfg.sentido = -cfg.sentido; salvarCfg();
      Serial.print(F("{\"OK\":\"SENTIDO\",\"valor\":")); Serial.print(cfg.sentido); Serial.println('}');
      break;

    case 'M':
      if (modo == MODO_SEM_ZERO) { alerta(F("ZERO_NAO_CALIBRADO"), F("USE_Z_PRIMEIRO")); break; }
      if (lerInteiros(arg, v, 1) != 1 || v[0] < 0 || v[0] > 90) {
        alerta(F("CMD_M_USO"), F("M0..M90")); break;
      }
      alvoManual = (float)v[0]; tManual = agora; modo = MODO_MANUAL;
      falhaMec = false; stalls = 0;
      Serial.print(F("{\"OK\":\"MANUAL\",\"Angulo\":")); Serial.print(v[0]); Serial.println('}');
      break;

    case 'F':
      if (modo != MODO_MANUAL) { alerta(F("CMD_F_SO_EM_MANUAL")); break; }
      if (lerInteiros(arg, v, 1) != 1 || v[0] < 0 || v[0] > 100) { alerta(F("CMD_F_USO"), F("F0..F100")); break; }
      fanManualPct = (uint8_t)v[0]; tManual = agora;
      Serial.print(F("{\"OK\":\"EXAUSTOR_MANUAL\",\"Pct\":")); Serial.print(v[0]); Serial.println('}');
      break;

    case 'A':
      if (modo == MODO_SEM_ZERO) { alerta(F("ZERO_NAO_CALIBRADO"), F("USE_Z_PRIMEIRO")); break; }
      modo = MODO_AUTO; fanManualPct = 0; falhaMec = false; stalls = 0; ultimoAlvoAuto = 0;
      Serial.println(F("{\"OK\":\"AUTO\"}"));
      break;

    case 'H':
      if (modo == MODO_SEM_ZERO) { alerta(F("ZERO_NAO_CALIBRADO"), F("USE_Z_PRIMEIRO")); break; }
      iniciarHoming();
      Serial.println(F("{\"OK\":\"HOMING_INICIADO\"}"));
      break;

    case '?':
      imprimirTelemetria(); imprimirConfig();
      break;

    default:
      alerta(F("COMANDO_DESCONHECIDO"), F("S Z I M F A H ?"));
  }
}

/* Aceita linhas terminadas por Enter e tambem comandos sem terminador (apos 200 ms). */
char linhaCmd[28]; uint8_t linhaN = 0; unsigned long tUltimoChar = 0;
void serialLer() {
  while (Serial.available()) {
    char c = Serial.read(); tUltimoChar = agora;
    if (c == '\n' || c == '\r') {
      if (linhaN) { linhaCmd[linhaN] = 0; linhaN = 0; processarComando(linhaCmd); }
    } else if (linhaN < sizeof(linhaCmd) - 1) linhaCmd[linhaN++] = c;
  }
  if (linhaN && (agora - tUltimoChar > 200)) { linhaCmd[linhaN] = 0; linhaN = 0; processarComando(linhaCmd); }
}

/* =============================================================================
   15) SINALIZACAO (LEDs + buzzer)
   ============================================================================= */
void sinalizar() {
  bool falha = (modo == MODO_SEM_ZERO) || !encoderOk || falhaMec;
  bool alarme = falha || failsafe;

  // Vermelho: falha = pisca rapido | fail-safe = pisca lento | ok = apagado
  bool verm = falha ? ((agora / 250) % 2) : (failsafe ? ((agora / 600) % 2) : false);
  // Verde: AUTO fixo | movendo/homing pisca | manual pisca lento
  bool verde;
  if (modo == MODO_AUTO)        verde = movendo ? ((agora / 400) % 2) : true;
  else if (modo == MODO_MANUAL) verde = ((agora / 1000) % 2);
  else if (modo == MODO_HOMING) verde = ((agora / 200) % 2);
  else                          verde = false;
  digitalWrite(PIN_LED_VERM, verm);
  digitalWrite(PIN_LED_VERDE, verde);

  static bool bipando = false;
  bool quer = BUZZER_ATIVO && alarme && ((agora % 5000UL) < 120UL);
  if (quer && !bipando)  { tone(PIN_BUZZER, 2000); bipando = true; }
  if (!quer && bipando)  { noTone(PIN_BUZZER);     bipando = false; }
}

/* =============================================================================
   16) SETUP E LOOP
   ============================================================================= */
void setup() {
  Serial.begin(115200);

#if defined(ESP32)
  Wire.begin(PIN_SDA, PIN_SCL);
#else
  Wire.begin();
  #if defined(WIRE_HAS_TIMEOUT)
    Wire.setWireTimeout(3000, true);     // nao trava se o barramento I2C cair
  #endif
#endif
#if CO2_SENSOR == CO2_SCD30
  Wire.setClock(100000);                 // SCD30 trabalha a 100 kHz
#else
  Wire.setClock(400000);
#endif

  carregarCfg();

  pinMode(PIN_MOT_A, OUTPUT); pinMode(PIN_MOT_B, OUTPUT); pinMode(PIN_EXAUSTOR, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT); pinMode(PIN_LED_VERDE, OUTPUT); pinMode(PIN_LED_VERM, OUTPUT);
  pwmIniciar();
  motorParar();
  exaustorAplicar(0);

  co2Iniciar();
  thIniciar();

  agora = millis(); tBoot = agora; tCo2Ok = agora; tThOk = agora; tEncoderOk = agora;

  // Primeira leitura do encoder (algumas tentativas)
  for (uint8_t i = 0; i < 5 && !encoderOk; i++) { atualizarAngulo(); if (!encoderOk) delay(10); }
  if (!encoderOk) { encoderOk = false; alerta(F("ENCODER_AS5600_SEM_RESPOSTA")); }
  else if (!as5600ImaOk()) alerta(F("IMA_NAO_DETECTADO_NO_AS5600"));

  Serial.println(F("{\"INFO\":\"DAMPER_CO2_10MM_INICIADO\",\"baud\":115200}"));
  imprimirConfig();

  // Homing: a partida SEMPRE fecha a valvula antes de qualquer outra acao
  if (cfg.zeroCalibrado) {
    iniciarHoming();
    Serial.println(F("{\"INFO\":\"HOMING_INICIADO\"}"));
  } else {
    modo = MODO_SEM_ZERO;
    alerta(F("ZERO_NAO_CALIBRADO"), F("FECHE_O_DISCO_E_ENVIE_Z"));
  }
}

void loop() {
  agora = millis();

  atualizarAngulo();       // posicao (AS5600)
  co2Atualizar();          // CO2 (UART/I2C, nao bloqueante)
  thAtualizar();           // temperatura/umidade
  serialLer();             // comandos
  supervisionar();         // fail-safe, expiracao do manual, rearme de falhas
  atualizarControle();     // decide alvo + exaustor conforme o modo
  atuadorLoop();           // executa a manobra em malha fechada

  if (aCada(tTelem, T_TELEMETRIA)) imprimirTelemetria();
  sinalizar();
}
