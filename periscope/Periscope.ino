// Arduino Uno + CNC Shield V3 X-axis only
// Same Timer1 architecture as your working code

/* Pin map -------------------------------------------------------------- */
#define STEP_X 2
#define DIR_X  5
#define EN_PIN 8

/* Motion parameters ---------------------------------------------------- */
const uint16_t BASE_TICK_US = 100;   // 10 kHz timer tick

const long FULL_STEPS_PER_REV = 200;
const long MICROSTEPS = 4;           // 1/4 microstepping
const long STEPS_PER_REV = FULL_STEPS_PER_REV * MICROSTEPS; // 800

/* State ---------------------------------------------------------------- */
volatile long pos = 0;
volatile bool motorRunning = false;

volatile uint16_t tickDivider = 10;
volatile uint16_t tickCounter = 0;

bool direction = true;
float speedRevPerSec = 1.0;

String inputString;

/* STEP pin mask for PORTD */
const uint8_t STEP_MASK = _BV(2);

/* ---------------- Timer1 ISR ---------------------------------------- */
ISR(TIMER1_COMPA_vect) {
  if (!motorRunning) return;

  tickCounter++;

  if (tickCounter >= tickDivider) {
    tickCounter = 0;

    PORTD |= STEP_MASK;
    delayMicroseconds(10);
    PORTD &= ~STEP_MASK;

    if (direction) pos++;
    else pos--;
  }
}

/* ---------------- update speed -------------------------------------- */
void updateSpeed(float revPerSec) {
  if (revPerSec <= 0) return;

  speedRevPerSec = revPerSec;

  float stepsPerSecond = speedRevPerSec * STEPS_PER_REV;

  float ticksPerStep = 1000000.0 / (stepsPerSecond * BASE_TICK_US);

  if (ticksPerStep < 1) ticksPerStep = 1;

  noInterrupts();
  tickDivider = (uint16_t)ticksPerStep;
  tickCounter = 0;
  interrupts();

  Serial.print("Speed set to ");
  Serial.print(speedRevPerSec, 3);
  Serial.println(" rev/s");
}

/* ---------------- setup ---------------------------------------------- */
void setup() {
  Serial.begin(115200);

  DDRD |= STEP_MASK;
  pinMode(DIR_X, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  digitalWrite(EN_PIN, LOW);
  digitalWrite(DIR_X, HIGH);

  updateSpeed(speedRevPerSec);

  noInterrupts();
  TCCR1A = 0;
  TCCR1B = _BV(WGM12) | _BV(CS10);
  OCR1A  = (F_CPU / 1000000UL) * BASE_TICK_US - 1;
  TIMSK1 = _BV(OCIE1A);
  interrupts();

  Serial.println("Stepper ready");
  Serial.println("Commands:");
  Serial.println("start");
  Serial.println("stop");
  Serial.println("speed <rev_per_sec>");
  Serial.println("dir 0");
  Serial.println("dir 1");
  Serial.println("zero");
}

/* ---------------- main loop ------------------------------------------ */
unsigned long lastReport = 0;

void loop() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      inputString.trim();

      if (inputString.equalsIgnoreCase("start")) {
        motorRunning = true;
        Serial.println("Started");
      }

      else if (inputString.equalsIgnoreCase("stop")) {
        motorRunning = false;
        Serial.println("Stopped");
      }

      else if (inputString.startsWith("speed")) {
        float newSpeed = inputString.substring(5).toFloat();
        updateSpeed(newSpeed);
      }

      else if (inputString.startsWith("dir")) {
        int d = inputString.substring(3).toInt();
        direction = d != 0;
        digitalWrite(DIR_X, direction ? HIGH : LOW);
        Serial.println(direction ? "Direction HIGH" : "Direction LOW");
      }

      else if (inputString.equalsIgnoreCase("zero")) {
        noInterrupts();
        pos = 0;
        interrupts();
        Serial.println("Zeroed");
      }

      inputString = "";
    } else {
      inputString += c;
    }
  }

  if (millis() - lastReport >= 5) {
    lastReport = millis();

    long p;
    noInterrupts();
    p = pos;
    interrupts();

    long wrappedSteps = p % STEPS_PER_REV;
    if (wrappedSteps < 0) wrappedSteps += STEPS_PER_REV;

    float angle = wrappedSteps * (360.0 / STEPS_PER_REV);

    Serial.print("A ");
    Serial.println(angle, 2);
  }
}
