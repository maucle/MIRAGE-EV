#include <math.h>

/* Pin map -------------------------------------------------------------- */
#define STEP_X 2
#define STEP_Y 3
#define STEP_Z 4
#define DIR_X  5
#define DIR_Y  6
#define DIR_Z  7
#define EN_PIN 8

/* Motion parameters ---------------------------------------------------- */
const uint16_t BASE_TICK_US = 100;    // 10kHz tick
volatile long pos[3] = {0, 0, 0};
volatile long tgt[3] = {0, 0, 0};
volatile bool busy = false;

volatile long rem[3];
volatile long absSteps[3];
volatile long majorSteps;

/* STEP-pin bit-masks for PORTD */
const uint8_t STEP_MASKS[3] = {_BV(2), _BV(3), _BV(4)};
const uint8_t DIR_PINS [3]  = {DIR_X, DIR_Y, DIR_Z};

/* demo state ----------------------------------------------------------- */
bool demoMode = false;
unsigned long demoStartTime = 0;
float circle_radius_deg = 8.0;
float circle_period = 800.0;
String patternMode = "circle";
String inputString;
long circleCenter[3] = {0,0,0};    // new circle center storage

/* inverse kinematics --------------------------------------------------- */
class Machine {
  public:
    double d, e, f, g;

    Machine(double _d, double _e, double _f, double _g) {
      d = _d; e = _e; f = _f; g = _g;
    }

    double theta(char leg, double hz, double nx, double ny) {
      double nmag = sqrt(nx*nx + ny*ny + 1);
      nx /= nmag;
      ny /= nmag;
      double nz = 1.0 / nmag;
      double y,x,z,mag,angle;

      switch (leg) {
        case 'A':
          y = d + (e/2.0) * (
            1 - (
              (nx*nx + 3*nz*nz + 3*nz)
              / (
                nz + 1
                - nx*nx
                + (pow(nx,4) - 3*nx*nx*ny*ny)
                  / ((nz + 1)*(nz + 1 - nx*nx))
              )
            )
          );
          z = hz + e*ny;
          mag = sqrt(y*y + z*z);
          angle = acos(y/mag) + acos(
            (mag*mag + f*f - g*g)/(2*mag*f)
          );
          break;
        case 'B':
          x = (sqrt(3.0)/2.0) * (
            e*(1 - (nx*nx + sqrt(3.0)*nx*ny)/(nz+1)) - d
          );
          y = x/sqrt(3.0);
          z = hz - (e/2.0)*(sqrt(3.0)*nx + ny);
          mag = sqrt(x*x + y*y + z*z);
          angle = acos((sqrt(3.0)*x + y)/(-2*mag)) + acos(
            (mag*mag + f*f - g*g)/(2*mag*f)
          );
          break;
        case 'C':
          x = (sqrt(3.0)/2.0) * (
            d - e*(1 - (nx*nx - sqrt(3.0)*nx*ny)/(nz+1))
          );
          y = -x/sqrt(3.0);
          z = hz + (e/2.0)*(sqrt(3.0)*nx - ny);
          mag = sqrt(x*x + y*y + z*z);
          angle = acos((sqrt(3.0)*x - y)/(2*mag)) + acos(
            (mag*mag + f*f - g*g)/(2*mag*f)
          );
          break;
        default:
          angle = 0;
      }
      return angle * 180.0/M_PI - 157.0;
    }
};

Machine m(48, 80, 44, 83.5);

/* ---------------- Timer1 ISR ---------------------------------------- */
ISR(TIMER1_COMPA_vect) {
  if (!busy) return;

  uint8_t stepOutHigh = 0;

  for (uint8_t ax = 0; ax < 3; ax++) {
    if (absSteps[ax] == 0) continue;
    rem[ax] += absSteps[ax];
    if (rem[ax] >= majorSteps) {
      rem[ax] -= majorSteps;
      if (tgt[ax] > pos[ax]) pos[ax]++;
      else if (tgt[ax] < pos[ax]) pos[ax]--;
      stepOutHigh |= STEP_MASKS[ax];
      absSteps[ax]--;
    }
  }

  if (stepOutHigh) {
    PORTD |=  stepOutHigh;
    delayMicroseconds(10);
    PORTD &= ~stepOutHigh;
  }

  if (absSteps[0]==0 && absSteps[1]==0 && absSteps[2]==0) busy = false;
}

/* ---------------- helpers -------------------------------------------- */
void start_move(long tx, long ty, long tz) {
  tgt[0] = tx;
  tgt[1] = ty;
  tgt[2] = tz;

  long delta[3] = { tx - pos[0], ty - pos[1], tz - pos[2] };

  for (uint8_t ax = 0; ax < 3; ax++) {
    digitalWrite(DIR_PINS[ax], delta[ax] >= 0 ? HIGH : LOW);
    absSteps[ax] = abs(delta[ax]);
  }

  majorSteps = max(absSteps[0], max(absSteps[1], absSteps[2]));
  for (uint8_t ax = 0; ax < 3; ax++) rem[ax] = majorSteps / 2;

  busy = true;
}

/* ---------------- setup ---------------------------------------------- */
void setup() {
  Serial.begin(115200);

  DDRD |= (_BV(2) | _BV(3) | _BV(4));
  pinMode(DIR_X, OUTPUT);
  pinMode(DIR_Y, OUTPUT);
  pinMode(DIR_Z, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);  // enable steppers

  noInterrupts();
  TCCR1A = 0;
  TCCR1B = _BV(WGM12) | _BV(CS10);
  OCR1A  = (F_CPU / 1000000UL) * BASE_TICK_US - 1;
  TIMSK1 = _BV(OCIE1A);
  interrupts();
}

/* ---------------- main loop ------------------------------------------ */
unsigned long lastReport = 0;

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      inputString.trim();
      if (inputString.equalsIgnoreCase("demo")) {
        demoMode = true;
        demoStartTime = millis();
        Serial.print("Demo mode started with pattern: ");
        Serial.println(patternMode);
      }
      else if (inputString.equalsIgnoreCase("stop")) {
        demoMode = false;
        Serial.println("Demo mode stopped.");
      }
      else if (inputString.startsWith("circle_goto")) {
        long cx, cy, cz;
        sscanf(inputString.c_str(), "circle_goto %ld %ld %ld", &cx, &cy, &cz);

        Serial.print("Going to: ");
        Serial.print(cx); Serial.print(", ");
        Serial.print(cy); Serial.print(", ");
        Serial.println(cz);

        start_move(cx, cy, cz);

        circleCenter[0] = cx;
        circleCenter[1] = cy;
        circleCenter[2] = cz;

        patternMode = "circle";
        circle_radius_deg = 0.1;
        circle_period = 100.0;
        demoMode = true;
        demoStartTime = millis();

        Serial.println("Starting 1° circle at 500ms around this coordinate.");
      }
      else if (inputString.startsWith("speed")) {
        int newSpeed = inputString.substring(5).toInt();
        if (newSpeed > 50) {
          circle_period = newSpeed;
          Serial.print("Demo speed updated: ");
          Serial.print(circle_period);
          Serial.println(" ms per pattern cycle.");
        }
      }
      else if (inputString.startsWith("radius")) {
        int newRadius = inputString.substring(6).toInt();
        if (newRadius >= 1 && newRadius <= 30) {
          circle_radius_deg = newRadius;
          Serial.print("Demo radius updated: ");
          Serial.print(circle_radius_deg);
          Serial.println(" degrees.");
        }
      }
      else if (inputString.startsWith("pattern")) {
        patternMode = inputString.substring(7);
        patternMode.trim();
        Serial.print("Pattern set to: ");
        Serial.println(patternMode);
      }
      else if (inputString.startsWith("G")) {
        long tx, ty, tz;
        sscanf(inputString.c_str(), "G %ld %ld %ld", &tx, &ty, &tz);
        start_move(tx, ty, tz);
      }
      inputString = "";
    } else {
      inputString += c;
    }
  }

  if (demoMode && !busy) {
    unsigned long now = millis();
    float phase = 2 * M_PI * (now - demoStartTime) / circle_period;

    float tilt_x = 0;
    float tilt_y = 0;

    if (patternMode.equalsIgnoreCase("circle")) {
      tilt_x = circle_radius_deg * cos(phase);
      tilt_y = circle_radius_deg * sin(phase);
    }

    double nx = tan(tilt_x * M_PI/180.0);
    double ny = tan(tilt_y * M_PI/180.0);
    double hz = 99;

    double thetaA = m.theta('A', hz, nx, ny);
    double thetaB = m.theta('B', hz, nx, ny);
    double thetaC = m.theta('C', hz, nx, ny);

    long stepsA = (long)(thetaA * 3200.0/360.0) + circleCenter[0];
    long stepsB = (long)(thetaB * 3200.0/360.0) + circleCenter[1];
    long stepsC = (long)(thetaC * 3200.0/360.0) + circleCenter[2];

    start_move(stepsA, stepsB, stepsC);

    if ((now - demoStartTime) >= circle_period) {
      demoStartTime = now;
    }
  }

  if (millis() - lastReport >= 5) {
    lastReport = millis();
    Serial.print("P ");
    Serial.print(pos[0]); Serial.print(' ');
    Serial.print(pos[1]); Serial.print(' ');
    Serial.println(pos[2]);
  }
}
