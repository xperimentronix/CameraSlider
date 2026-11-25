#include <AccelStepper.h>
#include <EEPROM.h>

// ==========================================
// DEFINICIÓN DE PINES (CNC Shield v4 / Arduino Nano)
// ==========================================
#define X_STEP_PIN      5
#define X_DIR_PIN       2
#define Y_STEP_PIN      6
#define Y_DIR_PIN       3

#define X_LIMIT_PIN     9   // Final de carrera eje X
#define Y_LIMIT_PIN     10  // Final de carrera eje Y

#define ENABLE_PIN      8   // Habilitación de drivers de la CNC shield (activo a nivel bajo)

// ==========================================
// DEFINES DE CONFIGURACIÓN Y VALORES POR DEFECTO
// ==========================================
#define DEF_POS_INIT_X  0       // posicion inicial por defecto X (pasos reales)
#define DEF_POS_INIT_Y  0       // posicion inicial por defecto Y (pasos reales)
#define DEF_POS_FINAL_X 20000   // posicion final por defecto X (pasos reales)
#define DEF_POS_FINAL_Y 20000   // posicion final por defecto Y (pasos reales)
#define DEF_SPEED       4000   // Velocidad real por defecto del perfil (pasos/s)
#define DEF_ACCEL       4000   // Aceleracion real por defecto del perfil (pasos/s^2)

// Valores reales maximos para el mapeo de los sliders de pantalla (minimo = 0).
// El slider manda 0..SLIDER_MAX por UART y se mapea a [0, estos maximos].
#define MAX_SPEED       4000   // Velocidad real maxima (pasos/s)
#define MAX_ACCEL       4000   // Aceleracion real maxima (pasos/s^2)

// Relacion de velocidad y aceleracion entre el eje Y y el eje X (escalado x1000).
// X es la referencia e Y = n*X, con n = ratio/1000.
// 1000 = X e Y iguales; 2000 = Y al doble que X; 500 = Y a la mitad que X.
// Mantiene el ajuste de velocidad/aceleracion compartido y compensa que los
// motores de X e Y tengan distinta resolucion de pasos.
#define RATIO_SPEED_XY  1000
#define RATIO_ACCEL_XY  1000

// Velocidad/aceleracion del homing en pasos/s y pasos/s^2 REALES (sin escalar; van directas a AccelStepper).
#define HOMING_SPEED    4000
#define HOMING_ACCEL    4000
#define HOMING_DIR_X    -1       // Dirección hacia el final de carrera (-1 o +1)
#define HOMING_DIR_Y    -1

// Si al iniciar el homing el final de carrera ya esta pulsado, el motor se aleja
// primero estos pasos (en sentido contrario al homing) y luego busca el switch.
#define BACKOFF_STEPS_X 4000L
#define BACKOFF_STEPS_Y 4000L

// Recorrido maximo de cada eje en pasos, desde home (0) hasta el extremo opuesto.
// No hay final de carrera en ese extremo: se usa como limite por software para no chocar.
// (Valor arbitrario por ahora; ajustar al recorrido real del montaje.)
#define MAX_POS_X       100000L
#define MAX_POS_Y       22000L
#define POST_HOMING_DELAY 500
#define POST_INIT_DELAY   500    // Tiempo de espera en milisegundos tras llegar a posición inicial
#define MOTOR_IDLE_TIMEOUT 30000 // ms sin movimiento tras los que se desactivan los drivers (enable OFF)

// Velocidad/aceleracion para ir a inicial/final y durante play, en unidades REALES (sin escalar).
#define HARDCODED_MOVE_SPEED 4000.0
#define HARDCODED_MOVE_ACCEL 4000.0

#define SCALE           1L       // Factor de escala para velocidad y aceleración (1 = escalado desactivado por ahora)
#define SLIDER_MAX      1000L    // Rango fijo de los sliders de pantalla (0..1000) que llega por UART

// ==========================================
// ESTRUCTURA DEL PERFIL DE MOVIMIENTO
// ==========================================
struct MovementProfile {
    uint32_t pos_inicial_x;
    uint32_t pos_inicial_y;
    uint32_t pos_final_x;
    uint32_t pos_final_y;
    int32_t velocidad_max;
    int32_t aceleracion;
    uint32_t checksum;
};

// ==========================================
// ENUMERACIÓN DE COMANDOS Y ESTADOS
// ==========================================
enum CommandId {
    CMD_UNKNOWN,
    CMD_CAL,
    CMD_LOADCFGPOS,
    CMD_RESET,
    CMD_GETCFGPOS,
    CMD_GETCFGSPAC,
    CMD_SETCFGSP,
    CMD_SETCFGAC,
    CMD_PLAYSTART,
    CMD_PLAYLOOP,
    CMD_PLAYPAUSE,
    CMD_PLAYSTOP,
    CMD_RECSAVE,
    CMD_GETCFGINITIAL,
    CMD_GETCFGFINAL,
    CMD_SETCFGINITX,
    CMD_SETCFGINITY,
    CMD_SETCFGFINALX,
    CMD_SETCFGFINALY,
    CMD_MOVEX,
    CMD_MOVEY,
    CMD_RPOSX,
    CMD_RPOSY,
    CMD_CLEARMEM
};

enum RunState {
    STATE_IDLE,
    STATE_HOMING_X_BACKOFF,
    STATE_HOMING_X,
    STATE_HOMING_Y_BACKOFF,
    STATE_HOMING_Y,
    STATE_HOMING_DELAY,
    STATE_MOVING_TO_INIT,
    STATE_MOVING_TO_FINAL,
    STATE_PLAY_INIT_MOVE,
    STATE_PLAY_WAIT,
    STATE_PLAY_START_RUN,
    STATE_PLAY_LOOP_RUN,
    STATE_MOVING_MANUAL
};

enum PlaySubsequentAction {
    ACTION_NONE,
    ACTION_PLAY_START,
    ACTION_PLAY_LOOP
};

// ==========================================
// VARIABLES GLOBALES
// ==========================================
MovementProfile temporalProfile;

AccelStepper stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);

// Movimiento sincronizado maestro-seguidor (ver startSyncedMove/runSyncedMove)
AccelStepper* masterStepper = &stepperX;  // eje que corre con rampa de AccelStepper
AccelStepper* slaveStepper  = &stepperY;  // eje que sigue al maestro
float slaveFollowRatio = 1.0f;            // dSeguidor/dMaestro (con signo)
long  slaveTarget = 0;                    // destino del seguidor (para congelar al final)

RunState currentState = STATE_IDLE;
PlaySubsequentAction playAction = ACTION_NONE;

unsigned long lastMotorActivity = 0;  // millis() del ultimo movimiento; base del timeout de apagado
bool motorsEnabled = false;           // estado actual del enable de los drivers

bool homingXDone = false;
bool homingYDone = false;
bool loopForward = true;

unsigned long actionTimer = 0;
String serialBuffer = "";

// Prototipos
void checkForSerialCommands();
void processCommand(String cmdLine);

// ==========================================
// FUNCIONES AUXILIARES DE ESCALADO
// ==========================================
float toFloatVal(int32_t scaledVal) {
    return (float)scaledVal / SCALE;
}

int32_t toScaledVal(float realVal) {
    return (int32_t)(realVal * SCALE);
}

// Mapea un valor de slider de pantalla (0..SLIDER_MAX) al rango real [0, maxReal].
// Lo usan los comandos UART que reciben velocidad, aceleracion y posiciones (inicio/fin).
long mapFromSlider(long sliderVal, long maxReal) {
    if (sliderVal < 0)          sliderVal = 0;
    if (sliderVal > SLIDER_MAX) sliderVal = SLIDER_MAX;
    return (long)((int32_t)sliderVal * maxReal / SLIDER_MAX);
}

// Mapeo inverso: valor real [0, maxReal] -> rango del slider (0..SLIDER_MAX).
// Lo usan los comandos get para que la pantalla muestre el valor en su escala 0-1000.
long mapToSlider(long realVal, long maxReal) {
    if (maxReal <= 0)        return 0;
    if (realVal < 0)         realVal = 0;
    if (realVal > maxReal)   realVal = maxReal;
    return (long)((int32_t)realVal * SLIDER_MAX / maxReal);
}

// Velocidad/aceleracion del perfil aplicadas por eje: X es la referencia e Y se escala por el ratio.
// X toma el valor del perfil tal cual; Y lo escala por el ratio (ratio/1000) => Y = n*X.
// AccelStepper no debe recibir 0 en velocidad ni aceleracion (lo dejaria sin
// moverse / sin rampa): se redondea cualquier valor menor que 1 a 1.
float noZero(float v) { return (v < 1.0f) ? 1.0f : v; }

float profileSpeedX() { return noZero(toFloatVal(temporalProfile.velocidad_max)); }
float profileSpeedY() { return noZero(toFloatVal(temporalProfile.velocidad_max) * (RATIO_SPEED_XY / 1000.0f)); }
float profileAccelX() { return noZero(toFloatVal(temporalProfile.aceleracion)); }
float profileAccelY() { return noZero(toFloatVal(temporalProfile.aceleracion) * (RATIO_ACCEL_XY / 1000.0f)); }

// ==========================================
// SINCRONIZACION DE EJES (maestro-seguidor, llegan a la vez)
// Un eje (el de mayor recorrido en pasos) corre con la rampa de AccelStepper;
// el otro NO lleva rampa propia: en cada ciclo copia la velocidad instantanea
// del maestro escalada por la razon de distancias (dSeguidor/dMaestro, con signo).
// Asi aceleran, cruceran y frenan juntos y terminan a la vez por construccion,
// sin depender de ninguna prediccion de tiempos. RATIO_* sigue aplicando como
// sesgo del eje a traves de profileSpeedX/Y y profileAccelX/Y.
// ==========================================
void startSyncedMove(long targetX, long targetY) {
    long dX = targetX - stepperX.currentPosition();
    long dY = targetY - stepperY.currentPosition();
    long adX = (dX < 0) ? -dX : dX;
    long adY = (dY < 0) ? -dY : dY;

    if (adX >= adY) {                  // X manda (recorrido mayor o igual)
        masterStepper = &stepperX;
        slaveStepper  = &stepperY;
        masterStepper->setMaxSpeed(profileSpeedX());
        masterStepper->setAcceleration(profileAccelX());
        slaveFollowRatio = (dX != 0) ? ((float)dY / (float)dX) : 0.0f;
        slaveTarget = targetY;
        masterStepper->moveTo(targetX);
    } else {                           // Y manda
        masterStepper = &stepperY;
        slaveStepper  = &stepperX;
        masterStepper->setMaxSpeed(profileSpeedY());
        masterStepper->setAcceleration(profileAccelY());
        slaveFollowRatio = (dY != 0) ? ((float)dX / (float)dY) : 0.0f;
        slaveTarget = targetX;
        masterStepper->moveTo(targetY);
    }
}

// Un paso del movimiento sincronizado. Devuelve true cuando el maestro llega
// (entonces se congela el seguidor en su destino para no acumular deriva).
bool runSyncedMove() {
    masterStepper->run();
    slaveStepper->setSpeed(masterStepper->speed() * slaveFollowRatio);
    slaveStepper->runSpeed();
    if (masterStepper->distanceToGo() == 0) {
        slaveStepper->setSpeed(0);
        slaveStepper->setCurrentPosition(slaveTarget);
        return true;
    }
    return false;
}

// ==========================================
// ACTUALIZACION EN CALIENTE DEL MOVIMIENTO DEL PERFIL
// Solo los movimientos generados desde el perfil temporal (play / play-loop,
// que corren con startSyncedMove) se pueden reconfigurar sobre la marcha.
// Si mientras corren llega una nueva velocidad, aceleracion o posicion final,
// se re-lanza el tramo en curso tomando la posicion ACTUAL como inicio y el
// destino (con los nuevos valores) como fin, sin esperar a terminar.
// Los movimientos hardcodeados (ir a inicial/final, play-init, manual) no se tocan.
// ==========================================
bool isProfileMoveActive() {
    return currentState == STATE_PLAY_START_RUN ||
           currentState == STATE_PLAY_LOOP_RUN;
}

void retargetActiveProfileMove() {
    // Destino del tramo en curso: en loop de vuelta se va a inicial; en el resto, a final.
    long tx, ty;
    if (currentState == STATE_PLAY_LOOP_RUN && !loopForward) {
        tx = clampPosX(temporalProfile.pos_inicial_x);
        ty = clampPosY(temporalProfile.pos_inicial_y);
    } else {
        tx = clampPosX(temporalProfile.pos_final_x);
        ty = clampPosY(temporalProfile.pos_final_y);
    }
    // startSyncedMove parte de currentPosition() y aplica profileSpeed/Accel actuales.
    startSyncedMove(tx, ty);
}

// ==========================================
// LIMITES DE RECORRIDO (software endstops)
// Suponiendo homing previo (home = 0), ningun destino puede salir de [0, MAX_POS].
// ==========================================
long clampPosX(long target) {
    if (target < 0)          return 0;
    if (target > MAX_POS_X)  return MAX_POS_X;
    return target;
}

long clampPosY(long target) {
    if (target < 0)          return 0;
    if (target > MAX_POS_Y)  return MAX_POS_Y;
    return target;
}

// ==========================================
// FUNCIONES DE CHECKSUM Y EEPROM
// ==========================================
uint32_t calculateChecksum(const MovementProfile &p) {
    const uint8_t *ptr = (const uint8_t*)&p;
    uint32_t sum = 0;
    for (size_t i = 0; i < sizeof(MovementProfile) - sizeof(uint32_t); i++) {
        sum += ptr[i];
    }
    return sum;
}

MovementProfile getDefaultProfile() {
    MovementProfile p;
    p.pos_inicial_x = DEF_POS_INIT_X;
    p.pos_inicial_y = DEF_POS_INIT_Y;
    p.pos_final_x = DEF_POS_FINAL_X;
    p.pos_final_y = DEF_POS_FINAL_Y;
    p.velocidad_max = toScaledVal(DEF_SPEED);
    p.aceleracion = toScaledVal(DEF_ACCEL);
    p.checksum = calculateChecksum(p);
    return p;
}

void loadProfileFromEEPROM(uint8_t index, MovementProfile &profile) {
    int addr = (index - 1) * sizeof(MovementProfile);
    EEPROM.get(addr, profile);
}

void saveProfileToEEPROM(uint8_t index, const MovementProfile &profile) {
    int addr = (index - 1) * sizeof(MovementProfile);
    MovementProfile pToSave = profile;
    pToSave.checksum = calculateChecksum(pToSave);
    EEPROM.put(addr, pToSave);
}

void initProfiles() {
    for (uint8_t i = 1; i <= 4; i++) {
        MovementProfile p;
        loadProfileFromEEPROM(i, p);
        if (p.checksum != calculateChecksum(p)) {
            saveProfileToEEPROM(i, getDefaultProfile());
        }
    }
    // Al arrancar, el perfil temporal usa SIEMPRE los valores por defecto (defines).
    // El perfil 1 solo se carga en el temporal cuando llega el comando loadcfgpos.
    temporalProfile = getDefaultProfile();
}

// ==========================================
// ENVÍO DE RESPUESTAS A LA NEXTION
// ==========================================
void sendNextionCommand(const char* cmd) {
    Serial.print(cmd);
    Serial.write(0xFF);
    Serial.write(0xFF);
    Serial.write(0xFF);
}

// ==========================================
// GESTION DEL ENABLE DE LOS DRIVERS
// Los motores solo se energizan al usarse; tras MOTOR_IDLE_TIMEOUT sin
// movimiento se desactivan para no quedar bloqueados todo el tiempo.
// ==========================================
void enableMotors() {
    digitalWrite(ENABLE_PIN, LOW);   // activo a nivel bajo: drivers ON
    motorsEnabled = true;
    lastMotorActivity = millis();
}

void disableMotors() {
    digitalWrite(ENABLE_PIN, HIGH);  // drivers OFF (libres)
    motorsEnabled = false;
}

// ==========================================
// FUNCIONES DE INICIO DE MOVIMIENTO (NO BLOQUEANTES)
// ==========================================
void startHomingAxisX() {
    stepperX.setMaxSpeed(HOMING_SPEED);
    stepperX.setAcceleration(HOMING_ACCEL);
    if (digitalRead(X_LIMIT_PIN) == LOW) {
        // El endstop ya esta pulsado: alejarse primero en sentido contrario al homing
        stepperX.move(-HOMING_DIR_X * BACKOFF_STEPS_X);
        currentState = STATE_HOMING_X_BACKOFF;
    } else {
        stepperX.moveTo(HOMING_DIR_X * 100000L);
        currentState = STATE_HOMING_X;
    }
}

void startHomingAxisY() {
    stepperY.setMaxSpeed(HOMING_SPEED);
    stepperY.setAcceleration(HOMING_ACCEL);
    if (digitalRead(Y_LIMIT_PIN) == LOW) {
        // El endstop ya esta pulsado: alejarse primero en sentido contrario al homing
        stepperY.move(-HOMING_DIR_Y * BACKOFF_STEPS_Y);
        currentState = STATE_HOMING_Y_BACKOFF;
    } else {
        stepperY.moveTo(HOMING_DIR_Y * 100000L);
        currentState = STATE_HOMING_Y;
    }
}

void performHoming() {
    // Homing secuencial: primero el eje X; al terminar X se lanza el de Y.
    // Cada eje: si su endstop ya esta pulsado al empezar, se aleja BACKOFF_STEPS
    // y despues hace el homing normal hasta activar el switch.
    enableMotors();
    homingXDone = false;
    homingYDone = false;
    startHomingAxisX();
}

void startMoveToInitial() {
    enableMotors();
    stepperX.setMaxSpeed(HARDCODED_MOVE_SPEED);
    stepperX.setAcceleration(HARDCODED_MOVE_ACCEL);
    stepperY.setMaxSpeed(HARDCODED_MOVE_SPEED);
    stepperY.setAcceleration(HARDCODED_MOVE_ACCEL);

    stepperX.moveTo(clampPosX(temporalProfile.pos_inicial_x));
    stepperY.moveTo(clampPosY(temporalProfile.pos_inicial_y));
    currentState = STATE_MOVING_TO_INIT;
}

void startMoveToFinal() {
    enableMotors();
    stepperX.setMaxSpeed(HARDCODED_MOVE_SPEED);
    stepperX.setAcceleration(HARDCODED_MOVE_ACCEL);
    stepperY.setMaxSpeed(HARDCODED_MOVE_SPEED);
    stepperY.setAcceleration(HARDCODED_MOVE_ACCEL);

    stepperX.moveTo(clampPosX(temporalProfile.pos_final_x));
    stepperY.moveTo(clampPosY(temporalProfile.pos_final_y));
    currentState = STATE_MOVING_TO_FINAL;
}

void startPlaySequence(PlaySubsequentAction action) {
    playAction = action;
    enableMotors();
    stepperX.setMaxSpeed(HARDCODED_MOVE_SPEED);
    stepperX.setAcceleration(HARDCODED_MOVE_ACCEL);
    stepperY.setMaxSpeed(HARDCODED_MOVE_SPEED);
    stepperY.setAcceleration(HARDCODED_MOVE_ACCEL);

    stepperX.moveTo(clampPosX(temporalProfile.pos_inicial_x));
    stepperY.moveTo(clampPosY(temporalProfile.pos_inicial_y));
    currentState = STATE_PLAY_INIT_MOVE;
}

// ==========================================
// MAPEO DE COMANDOS STRING A ENUM
// ==========================================
CommandId getCommandId(String cmd) {
    if (cmd == "cal")           return CMD_CAL;
    if (cmd == "loadcfgpos")    return CMD_LOADCFGPOS;
    if (cmd == "reset")         return CMD_RESET;
    if (cmd == "getcfgpos")     return CMD_GETCFGPOS;
    if (cmd == "getcfgspac")    return CMD_GETCFGSPAC;
    if (cmd == "setcfgsp")      return CMD_SETCFGSP;
    if (cmd == "setcfgac")      return CMD_SETCFGAC;
    if (cmd == "playstart")     return CMD_PLAYSTART;
    if (cmd == "playloop")      return CMD_PLAYLOOP;
    if (cmd == "playpause")     return CMD_PLAYPAUSE;
    if (cmd == "playstop")      return CMD_PLAYSTOP;
    if (cmd == "recsave")       return CMD_RECSAVE;
    if (cmd == "getcfginitial") return CMD_GETCFGINITIAL;
    if (cmd == "getcfgfinal")   return CMD_GETCFGFINAL;
    if (cmd == "setcfginitx")   return CMD_SETCFGINITX;
    if (cmd == "setcfginity")   return CMD_SETCFGINITY;
    if (cmd == "setcfgfinalx")  return CMD_SETCFGFINALX;
    if (cmd == "setcfgfinaly")  return CMD_SETCFGFINALY;
    if (cmd == "movex")         return CMD_MOVEX;
    if (cmd == "movey")         return CMD_MOVEY;
    if (cmd == "rposx")         return CMD_RPOSX;
    if (cmd == "rposy")         return CMD_RPOSY;
    if (cmd == "clearmem")      return CMD_CLEARMEM;
    return CMD_UNKNOWN;
}

// ==========================================
// PROCESAMIENTO DE COMANDOS UART
// ==========================================
void processCommand(String cmdLine) {
    cmdLine.trim();
    if (cmdLine.length() == 0) return;

    int spaceIndex = cmdLine.indexOf(' ');
    String command = "";
    String paramStr = "";

    if (spaceIndex != -1) {
        command = cmdLine.substring(0, spaceIndex);
        paramStr = cmdLine.substring(spaceIndex + 1);
    } else {
        command = cmdLine;
    }

    switch (getCommandId(command)) {
        case CMD_CAL:
            performHoming();
            break;

        case CMD_LOADCFGPOS: {
            uint8_t n = paramStr.toInt();
            if (n >= 1 && n <= 4) {
                MovementProfile p;
                loadProfileFromEEPROM(n, p);
                if (p.checksum == calculateChecksum(p)) {
                    temporalProfile = p;                 // perfil valido
                } else {
                    temporalProfile = getDefaultProfile(); // corrupto -> valores por defecto
                }
            }
            break;
        }

        case CMD_RESET:
            //sendNextionCommand("rest");
            Serial.flush();  // asegura que "rest" + 0xFF 0xFF 0xFF se envíe por completo
            asm volatile ("jmp 0");
            break;

        case CMD_GETCFGPOS: {
            char buf[50];
            long sx = mapToSlider(temporalProfile.pos_inicial_x, MAX_POS_X);
            long sy = mapToSlider(temporalProfile.pos_inicial_y, MAX_POS_Y);
            long ex = mapToSlider(temporalProfile.pos_final_x,   MAX_POS_X);
            long ey = mapToSlider(temporalProfile.pos_final_y,   MAX_POS_Y);
            sprintf(buf, "nSx.val=%ld", sx);
            sendNextionCommand(buf);
            sprintf(buf, "nSy.val=%ld", sy);
            sendNextionCommand(buf);
            sprintf(buf, "nEx.val=%ld", ex);
            sendNextionCommand(buf);
            sprintf(buf, "nEy.val=%ld", ey);
            sendNextionCommand(buf);
            break;
        }

        case CMD_GETCFGSPAC: {
            char buf[50];
            long sp = mapToSlider(temporalProfile.velocidad_max, MAX_SPEED);
            long ac = mapToSlider(temporalProfile.aceleracion, MAX_ACCEL);
            sprintf(buf, "sSpeed.val=%ld", sp);
            sendNextionCommand(buf);
            sprintf(buf, "nSpeed.val=%ld", sp);
            sendNextionCommand(buf);
            sprintf(buf, "sAccel.val=%ld", ac);
            sendNextionCommand(buf);
            sprintf(buf, "nAccel.val=%ld", ac);
            sendNextionCommand(buf);
            break;
        }

        case CMD_SETCFGSP:
            temporalProfile.velocidad_max = mapFromSlider(paramStr.toInt(), MAX_SPEED);
            // En caliente: si hay un movimiento del perfil en curso, aplicar ya la nueva velocidad.
            if (isProfileMoveActive()) retargetActiveProfileMove();
            break;

        case CMD_SETCFGAC:
            temporalProfile.aceleracion = mapFromSlider(paramStr.toInt(), MAX_ACCEL);
            // En caliente: si hay un movimiento del perfil en curso, aplicar ya la nueva aceleracion.
            if (isProfileMoveActive()) retargetActiveProfileMove();
            break;

        case CMD_PLAYSTART:
            startPlaySequence(ACTION_PLAY_START);
            break;

        case CMD_PLAYLOOP:
            startPlaySequence(ACTION_PLAY_LOOP);
            break;

        case CMD_PLAYPAUSE:
            stepperX.stop();
            stepperY.stop();
            currentState = STATE_IDLE;
            break;

        case CMD_PLAYSTOP:
            // Solo detiene el movimiento; NO regresa a la posicion inicial.
            stepperX.stop();
            stepperY.stop();
            currentState = STATE_IDLE;
            break;

        case CMD_RECSAVE: {
            uint8_t n = paramStr.toInt();
            if (n >= 1 && n <= 4) {
                saveProfileToEEPROM(n, temporalProfile);
            }
            break;
        }

        case CMD_GETCFGINITIAL: {
            char buf[50];
            long px = mapToSlider(temporalProfile.pos_inicial_x, MAX_POS_X);
            long py = mapToSlider(temporalProfile.pos_inicial_y, MAX_POS_Y);
            // Slider invertido (1000 - valor); el numerico conserva el valor real
            sprintf(buf, "sX.val=%ld", 1000 - px);
            sendNextionCommand(buf);
            sprintf(buf, "nX.val=%ld", px);
            sendNextionCommand(buf);
            sprintf(buf, "sY.val=%ld", 1000 - py);
            sendNextionCommand(buf);
            sprintf(buf, "nY.val=%ld", py);
            sendNextionCommand(buf);
            break;
        }

        case CMD_GETCFGFINAL: {
            char buf[50];
            long px = mapToSlider(temporalProfile.pos_final_x, MAX_POS_X);
            long py = mapToSlider(temporalProfile.pos_final_y, MAX_POS_Y);
            // Slider invertido (1000 - valor); el numerico conserva el valor real
            sprintf(buf, "sX.val=%ld", 1000 - px);
            sendNextionCommand(buf);
            sprintf(buf, "nX.val=%ld", px);
            sendNextionCommand(buf);
            sprintf(buf, "sY.val=%ld", 1000 - py);
            sendNextionCommand(buf);
            sprintf(buf, "nY.val=%ld", py);
            sendNextionCommand(buf);
            break;
        }

        case CMD_SETCFGINITX:
            temporalProfile.pos_inicial_x = mapFromSlider(paramStr.toInt(), MAX_POS_X);
            startMoveToInitial();
            break;

        case CMD_SETCFGINITY:
            temporalProfile.pos_inicial_y = mapFromSlider(paramStr.toInt(), MAX_POS_Y);
            startMoveToInitial();
            break;

        case CMD_SETCFGFINALX:
            temporalProfile.pos_final_x = mapFromSlider(paramStr.toInt(), MAX_POS_X);
            // En play: redirigir el movimiento del perfil al nuevo final sin parar.
            // Fuera de play: preview hardcodeado yendo a la posicion final.
            if (isProfileMoveActive()) retargetActiveProfileMove();
            else startMoveToFinal();
            break;

        case CMD_SETCFGFINALY:
            temporalProfile.pos_final_y = mapFromSlider(paramStr.toInt(), MAX_POS_Y);
            // En play: redirigir el movimiento del perfil al nuevo final sin parar.
            // Fuera de play: preview hardcodeado yendo a la posicion final.
            if (isProfileMoveActive()) retargetActiveProfileMove();
            else startMoveToFinal();
            break;

        case CMD_MOVEX: {
            // Incremento relativo de X (val puede ser positivo o negativo)
            long delta = paramStr.toInt();
            enableMotors();
            stepperX.setMaxSpeed(profileSpeedX());
            stepperX.setAcceleration(profileAccelX());
            stepperX.moveTo(clampPosX(stepperX.currentPosition() + delta));
            currentState = STATE_MOVING_MANUAL;
            break;
        }

        case CMD_MOVEY: {
            // Incremento relativo de Y (val puede ser positivo o negativo)
            long delta = paramStr.toInt();
            enableMotors();
            stepperY.setMaxSpeed(profileSpeedY());
            stepperY.setAcceleration(profileAccelY());
            stepperY.moveTo(clampPosY(stepperY.currentPosition() + delta));
            currentState = STATE_MOVING_MANUAL;
            break;
        }

        case CMD_RPOSX:
            // Respuesta para un usuario por terminal: numero entero + '\n' (una respuesta por linea)
            Serial.print(stepperX.currentPosition());
            Serial.print('\n');
            break;

        case CMD_RPOSY:
            Serial.print(stepperY.currentPosition());
            Serial.print('\n');
            break;

        case CMD_CLEARMEM: {
            // Rellena con 0xFF solo la zona de EEPROM que ocupan los 4 perfiles.
            // Tras esto, al arrancar initProfiles detecta checksum invalido y
            // reescribe los 4 perfiles con los valores por defecto.
            int total = 4 * sizeof(MovementProfile);
            for (int i = 0; i < total; i++) {
                EEPROM.update(i, 0xFF);
            }
            Serial.print(F("memclr\n"));
            break;
        }

        case CMD_UNKNOWN:
        default:
            break;
    }
}

void checkForSerialCommands() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\r') {
            // Ignorar CR: soporta terminacion '\n' y '\r\n'
            continue;
        }
        if (c == '\n') {
            if (serialBuffer.length() > 0) {
                processCommand(serialBuffer);
                serialBuffer = "";
            }
        } else {
            serialBuffer += c;
        }
    }
}

// ==========================================
// CONFIGURACIÓN Y BUCLE PRINCIPAL (MÁQUINA DE ESTADOS)
// ==========================================
void setup() {
    Serial.begin(9600);

    pinMode(X_LIMIT_PIN, INPUT_PULLUP);
    pinMode(Y_LIMIT_PIN, INPUT_PULLUP);

    // Drivers de la CNC shield (EN activo a nivel bajo). Arrancan DESACTIVADOS:
    // solo se activan al mover y se apagan tras MOTOR_IDLE_TIMEOUT sin movimiento.
    pinMode(ENABLE_PIN, OUTPUT);
    disableMotors();
    lastMotorActivity = millis();

    initProfiles();

    //inicializa pantalla
    sendNextionCommand("rest");
}

void loop() {
    // 1. Siempre escuchar el puerto serie
    checkForSerialCommands();

    // 2. Máquina de estados no bloqueante para gestionar motores y tiempos
    switch (currentState) {
        case STATE_IDLE:
            // Sin acción activa, esperando comandos
            break;

        case STATE_HOMING_X_BACKOFF:
            // Fase 0 (X): el endstop estaba pulsado; alejarse antes de buscar el switch
            stepperX.run();
            if (stepperX.distanceToGo() == 0) {
                stepperX.moveTo(HOMING_DIR_X * 100000L);
                currentState = STATE_HOMING_X;
            }
            break;

        case STATE_HOMING_X:
            // Fase 1: homing del eje X hacia el final de carrera
            if (digitalRead(X_LIMIT_PIN) == LOW) {
                stepperX.stop();
                stepperX.setCurrentPosition(0);
                homingXDone = true;
                // X ya esta en home: ahora se lanza el homing de Y
                startHomingAxisY();
            } else {
                stepperX.run();
            }
            break;

        case STATE_HOMING_Y_BACKOFF:
            // Fase 0 (Y): el endstop estaba pulsado; alejarse antes de buscar el switch
            stepperY.run();
            if (stepperY.distanceToGo() == 0) {
                stepperY.moveTo(HOMING_DIR_Y * 100000L);
                currentState = STATE_HOMING_Y;
            }
            break;

        case STATE_HOMING_Y:
            // Fase 2: homing del eje Y hacia el final de carrera (solo tras completar X)
            if (digitalRead(Y_LIMIT_PIN) == LOW) {
                stepperY.stop();
                stepperY.setCurrentPosition(0);
                homingYDone = true;
                actionTimer = millis();
                currentState = STATE_HOMING_DELAY;
            } else {
                stepperY.run();
            }
            break;

        case STATE_HOMING_DELAY:
            if (millis() - actionTimer >= POST_HOMING_DELAY) {
                currentState = STATE_IDLE;
                sendNextionCommand("homingDone.val=1");
                sendNextionCommand("bGo.pco=0");
                sendNextionCommand("bGo.pco2=65535");
                sendNextionCommand("bGo.bco2=1024");
            }
            break;

        case STATE_MOVING_TO_INIT:
        case STATE_MOVING_TO_FINAL:
        case STATE_MOVING_MANUAL:
            stepperX.run();
            stepperY.run();
            if (stepperX.distanceToGo() == 0 && stepperY.distanceToGo() == 0) {
                currentState = STATE_IDLE;
            }
            break;

        case STATE_PLAY_INIT_MOVE:
            stepperX.run();
            stepperY.run();
            if (stepperX.distanceToGo() == 0 && stepperY.distanceToGo() == 0) {
                actionTimer = millis();
                currentState = STATE_PLAY_WAIT;
            }
            break;

        case STATE_PLAY_WAIT:
            if (millis() - actionTimer >= POST_INIT_DELAY) {
                // Perfil sincronizado: ambos ejes arrancan, cruceran y frenan a la vez
                long txf = clampPosX(temporalProfile.pos_final_x);
                long tyf = clampPosY(temporalProfile.pos_final_y);
                startSyncedMove(txf, tyf);

                if (playAction == ACTION_PLAY_START) {
                    currentState = STATE_PLAY_START_RUN;
                } else if (playAction == ACTION_PLAY_LOOP) {
                    loopForward = true;
                    currentState = STATE_PLAY_LOOP_RUN;
                }
            }
            break;

        case STATE_PLAY_START_RUN:
            if (runSyncedMove()) {
                currentState = STATE_IDLE;
            }
            break;

        case STATE_PLAY_LOOP_RUN:
            if (runSyncedMove()) {
                if (loopForward) {
                    loopForward = false;
                    startSyncedMove(clampPosX(temporalProfile.pos_inicial_x),
                                    clampPosY(temporalProfile.pos_inicial_y));
                } else {
                    loopForward = true;
                    startSyncedMove(clampPosX(temporalProfile.pos_final_x),
                                    clampPosY(temporalProfile.pos_final_y));
                }
            }
            break;
    }

    // 3. Gestion del enable: mantener vivo el timeout mientras hay movimiento,
    //    y desactivar los drivers tras MOTOR_IDLE_TIMEOUT en reposo.
    if (currentState != STATE_IDLE) {
        lastMotorActivity = millis();
    } else if (motorsEnabled && (millis() - lastMotorActivity >= MOTOR_IDLE_TIMEOUT)) {
        disableMotors();
    }
}
