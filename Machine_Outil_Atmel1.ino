//constantes

#define BAUDRATE (9600)
#define MAX_BUFFER (64)
#define NUMBER_OF_AXIS (4)
#define MAX_FEEDRATE (620)
#define MIN_FEEDRATE (1)
#define DEFAULT_FEEDRATE (100)
#define MM_PER_SEGMENT  (1)
#define error_pin  (2)
const int steppers_enable_pin = 4;
#define spindlepin (3)
#define heat1pin (5)
#define heat2pin (6)
volatile int limit = 0;


// STRUCTS

typedef struct {
  long delta;
  long absdelta;
  long over;
  int STEPS_PER_MM;
} Axis;


typedef struct {
  int step_pin;
  int dir_pin;
} Stepper;


//GLOBALS

Axis AX[NUMBER_OF_AXIS];
Stepper stepper[NUMBER_OF_AXIS];

char buffer[MAX_BUFFER];
int buffer_count;
float feedrate = DEFAULT_FEEDRATE;
long step_delay;

float px = 0, py = 0, pz = 0, pe = 0;

// settings
char mode_abs = 1;
int abs_mode_e = 1;
long line_number = 0;



String temptext;

// FUNCTIONS
void where(int notall = 0);

void pause(long ms) {
  delay(ms / 1000);
  delayMicroseconds(ms % 1000);
}

void feed_rate(float new_feedrate) {
  if (feedrate == new_feedrate) return;

  if (new_feedrate > MAX_FEEDRATE || new_feedrate < MIN_FEEDRATE) {
    Serial.print(F("New feedrate out of range"));
    Serial.println(F("steps/s."));
    return;
  }
  step_delay = MAX_FEEDRATE / new_feedrate;
  feedrate = new_feedrate;
}


void position(float new_x_position, float new_y_position, float new_z_position, float new_e_position) {
  px = new_x_position;
  py = new_y_position;
  pz = new_z_position;
  pe = new_e_position;
}


// Step
void onestep(int motor) {
  if (limit != 1) {
    digitalWrite(stepper[motor].step_pin, HIGH);
    digitalWrite(stepper[motor].step_pin, LOW);
  }
}

//LINE
void line(float newx, float newy, float newz, float newe) {
  AX[0].delta = (newx - px) * AX[0].STEPS_PER_MM;
  AX[1].delta = (newy - py) * AX[1].STEPS_PER_MM;
  AX[2].delta = (newz - pz) * AX[2].STEPS_PER_MM;
  AX[3].delta = (newe - pe) * AX[3].STEPS_PER_MM;

  long i, j, maxsteps = 0;

  for (i = 0; i < NUMBER_OF_AXIS; ++i) {
    AX[i].absdelta = abs(AX[i].delta);
    if ( maxsteps < AX[i].absdelta ) maxsteps = AX[i].absdelta;
    // set the direction once per movement
    digitalWrite(stepper[i].dir_pin, AX[i].delta > 0 ? HIGH : LOW);

  }
  for (i = 0; i < NUMBER_OF_AXIS; ++i) {
    AX[i].over = maxsteps / 2;
  }

  //long dt = MAX_FEEDRATE/5000;


  long mindt = 60000000 / (AX[0].STEPS_PER_MM * feedrate);
  long dt = mindt;
  if (feedrate > DEFAULT_FEEDRATE) {
    dt = 60000000 / (AX[0].STEPS_PER_MM * DEFAULT_FEEDRATE);
  }
  long accel = 1;
  long steps_to_accel = dt - step_delay;
  if (steps_to_accel > maxsteps / 2 )
    steps_to_accel = maxsteps / 2;

  long steps_to_decel = maxsteps - steps_to_accel;

  for ( i = 0; i < maxsteps; ++i ) {
    for (j = 0; j < NUMBER_OF_AXIS; ++j) {
      AX[j].over += AX[j].absdelta;
      if (AX[j].over >= maxsteps) {
        AX[j].over -= maxsteps;
        onestep(j);
        if (limit == 1) {
          Serial.println("Limit sensor active");
          return;
        }
      }
    }

    if (i < steps_to_accel && dt > mindt) {
      dt -= accel;
    }
    if (i >= steps_to_decel) {
      dt += accel;
    }
    delayMicroseconds(dt);
  }


  position(newx, newy, newz, newe);

}



float asin(float c) {
  float out;
  out = ((c + (c * c * c) / 6 + (3 * c * c * c * c * c) / 40 + (5 * c * c * c * c * c * c * c) / 112 + (35 * c * c * c * c * c * c * c * c * c) / 1152 + (c * c * c * c * c * c * c * c * c * c * c * 0.022) + (c * c * c * c * c * c * c * c * c * c * c * c * c * .0173) + (c * c * c * c * c * c * c * c * c * c * c * c * c * c * c * .0139) + (c * c * c * c * c * c * c * c * c * c * c * c * c * c * c * c * c * 0.0115) + (c * c * c * c * c * c * c * c * c * c * c * c * c * c * c * c * c * c * c * 0.01)));
  //asin
  if (c >= .96 && c < .97) {
    out = 1.287 + (3.82 * (c - .96));
  }
  if (c >= .97 && c < .98) {
    out = (1.325 + 4.5 * (c - .97)); // arcsin
  }
  if (c >= .98 && c < .99) {
    out = (1.37 + 6 * (c - .98));
  }
  if (c >= .99 && c <= 1) {
    out = (1.43 + 14 * (c - .99));
  }
  return out;
}
float arccos(float c) {
  float out;
  out = asin(sqrt(1 - c * c));
  return out;
}

float atan(float c) {
  float out;
  out = asin(c / (sqrt(1 + c * c)));
  return out;
}


void triangulate(float Ax, float Ay, float Bx, float By, float AB, float AC, float BC, float &Cx1, float &Cy1, float &Cx2, float &Cy2) {
  float cos1 = (AC * AC + AB * AB - BC * BC) / (2 * AC * AB);
  float sin1 = sqrt(1 - cos1 * cos1);
  float B1x, B1y;

  B1x = Bx - Ax;
  B1y = By - Ay;

  B1x *= AC / AB;
  B1y *= AC / AB;

  Cx1 = B1x * cos1 + B1y * sin1 + Ax;
  Cy1 = B1y * cos1 - B1x * sin1 + Ay;
  Cx2 = B1x * cos1 - B1y * sin1 + Ax;
  Cy2 = B1y * cos1 + B1x * sin1 + Ay;
}



float dist(float x1, float y1, float x2, float y2 ) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}



static float atan3(float dy, float dx) {
  float a = atan2(dy, dx);
  if (a < 0) a = (PI * 2.0) + a;
  return a;
}

float getq(float startx, float starty, float endx, float endy, float centerx, float centery) {
  float xstartvector = startx - centerx;
  float ystartvector = starty - centery;
  float xendvector = endx - centerx;
  float yendvector = endy - centery;
  float tx = sqrt(xstartvector * xstartvector + ystartvector * ystartvector);
  float ty = sqrt(xendvector * xendvector + yendvector * yendvector);
  PRINT("is it the radius ", String(tx), 1);
  PRINT("is it the radius ", String(ty), 1);

  float cosq = (xstartvector * xendvector + ystartvector * yendvector) / (tx * ty);
  PRINT("cosq ", String(cosq), 1);
  float q = arccos(cosq);
  PRINT("TETA rad ", String(q), 1);
  float deg = q / 0.01745329252;
  PRINT(" TETA deg", String(deg), 1);
  return q;
}
float getradius(float centerx, float centery, float pointx, float pointy) {
  float tx = pointx - centerx;
  float ty = pointy - centery;
  float radius = sqrt(tx * tx + ty * ty);
  return radius;
}





static void arc(float cx, float cy, float x, float y, float dir) {
  float startx = px;
  float starty = py;
  float centerx = px + cx;
  float centery = py + cy;
  PRINT("centerx ", String(centerx), 0);
  PRINT(" centery ", String(centery), 1);
  float radius = getradius(centerx, centery, startx, starty);
  PRINT("radius ", String(radius), 1);
  float q = getq(startx, starty, x, y, centerx, centery);

  float len;
  q > 0 ? len = abs(q) * radius : x != px ? len = radius * PI : len = 2 * radius * PI;

  int i, segments = floor( len * MM_PER_SEGMENT );
  PRINT("len ", String(len), 1);
  PRINT("segments ", String(segments), 1);
  for (i = 0; i < segments; ++i) {
    //if(dir>0 ){

    float Solution1x, Solution1y, Solution2x, Solution2y;

    triangulate(centerx, centery, px, py, radius, radius, MM_PER_SEGMENT, Solution1x, Solution1y, Solution2x, Solution2y);
    String temptext = "Point";
    temptext += i;
    temptext += ".1";
    Serial.print(temptext);
    Serial.print("X1: ");
    Serial.print(Solution1x);
    Serial.print(" Y1: ");
    Serial.print(Solution1y);

    temptext = " Point";
    temptext += i;
    temptext += ".2";
    Serial.print(temptext);
    Serial.print("X2: ");
    Serial.print(Solution2x);
    Serial.print(" Y2: ");
    Serial.println(Solution2y);
    if (dir < 1) {
      line(Solution1x, Solution1y, pz, pe);
    } else {
      line(Solution2x, Solution2y, pz, pe);
    }
    where(1);
  }
}

float parsenumber(char code, float val) {
  char *ptr = buffer;
  while ((long)ptr > 1 && (*ptr) && (long)ptr < (long)buffer + buffer_count) {
    if (*ptr == code) {
      return atof(ptr + 1);
    }
    ptr = strchr(ptr, ' ') + 1;
  }
  return val;
}


void PRINT(String text, String var, int ln) {
  Serial.print(text);
  ln ? Serial.println(var) : Serial.print(var);
}


//CURRENT POSITION
void where(int notall = 0) {
  PRINT("X ", String(px), 0);
  PRINT(" Y ", String(py), 0);
  PRINT(" Z ", String(pz), 0);
  PRINT(" E ", String(pe), 1);
  if (notall == 0) {
    PRINT("F ", String(feedrate), 1);
    mode_abs ? temptext = "ABSOLUTE" : temptext = "RELATIVE";
    PRINT("X Y Z ", temptext, 1);
    abs_mode_e ? temptext = "ABSOLUTE" : temptext = "RELATIVE";
    PRINT("E AXIS ", temptext, 1);
  }
}


/**
   display helpful information
*/
void help() {
  Serial.println(F("Commands:"));
  Serial.println(F("G00/G01 [X/Y/Z/E(steps)] [F(feedrate)]; - linear move"));
  Serial.println(F("G04 P[seconds]; - delay"));
  Serial.println(F("G90; - absolute mode"));
  Serial.println(F("G91; - relative mode"));
  Serial.println(F("G92 [X/Y/Z/E(steps)]; - change logical position"));
  Serial.println(F("M3; - Allumer Spindle"));
  Serial.println(F("M4; - Stop Spindle"));
  Serial.println(F("M5; - Z 12V tool On"));
  Serial.println(F("M6; - Z 12V tool Off"));

  Serial.println(F("M7; - table 12V tool On"));
  Serial.println(F("M8; - table 12V tool Off"));
  Serial.println(F("M17; - Activer motors"));
  Serial.println(F("M18; - Desactiver les moteurs"));
  Serial.println(F("M100; - Aide"));
  Serial.println(F("M114; - Position ? "));
  Serial.println(F("Grbl v1.Xx ['$' for help]."));

}

void processCommand() {
  int cmd = parsenumber('G', -1);
  switch (cmd) {
    case  0: {
        feed_rate(MAX_FEEDRATE);
        line( parsenumber('X', (mode_abs ? px : 0)) + (mode_abs ? 0 : px),
              parsenumber('Y', (mode_abs ? py : 0)) + (mode_abs ? 0 : py),
              parsenumber('Z', (mode_abs ? pz : 0)) + (mode_abs ? 0 : pz),
              parsenumber('E', (abs_mode_e ? pe : 0)) + (abs_mode_e ? 0 : pe) );
        break;
        feed_rate(feedrate);
      }
    case  1: { // line
        feed_rate(parsenumber('F', feedrate));
        line( parsenumber('X', (mode_abs ? px : 0)) + (mode_abs ? 0 : px),
              parsenumber('Y', (mode_abs ? py : 0)) + (mode_abs ? 0 : py),
              parsenumber('Z', (mode_abs ? pz : 0)) + (mode_abs ? 0 : pz),
              parsenumber('E', (abs_mode_e ? pe : 0)) + (abs_mode_e ? 0 : pe) );
        break;
      }
    case  2:  // arc
    case 3: {  // arc
        feed_rate(parsenumber('F', feedrate));
        arc(parsenumber('I', (mode_abs ? px : 0)) + (mode_abs ? 0 : px),
            parsenumber('J', (mode_abs ? py : 0)) + (mode_abs ? 0 : py),
            parsenumber('X', (mode_abs ? px : 0)) + (mode_abs ? 0 : px),
            parsenumber('Y', (mode_abs ? py : 0)) + (mode_abs ? 0 : py),
            (cmd == 2) ? -1 : 1);
        break;
      }
    case  4:  pause(parsenumber('P', 0) * 1000);  break; // dwell
    case 90:  mode_abs = 1;  break; // absolute mode
    case 91:  mode_abs = 0;  break; // relative mode
    case 92:  // set logical position
      position( parsenumber('X', 0),
                parsenumber('Y', 0),
                parsenumber('Z', 0),
                parsenumber('E', 0) );
      break;
    default:  break;
  }

  cmd = parsenumber('M', -1);
  switch (cmd) {
    case  3:  digitalWrite(9, HIGH);  break;
    case  4:  digitalWrite(9, LOW);  break;

    case  5:  heat1(1);  break;
    case  6:  heat1(0);  break;

    case  7:  heattable(1);  break;
    case  8:  heattable(0);  break;

    case  17:  motor_enable();  break;
    case  18:  motor_disable();  break;
    case 100:  help();  break;
    case 114:  where();  break;
    default:  break;
  }
}


void ready() {
  buffer_count = 0;
}
void spindle(int onoff) {
  if (onoff == 0) {
    digitalWrite(spindlepin, LOW);
  } else {
    digitalWrite(spindlepin, HIGH);
  }
}
void heat1(int onoff) {
  digitalWrite(heat1pin, onoff);
}
void heattable(int onoff) {
  digitalWrite(heat2pin, onoff);
}

void motor_setup() {
  stepper[0].dir_pin = 13;
  stepper[0].step_pin = 12;
  AX[0].STEPS_PER_MM = 640;

  stepper[1].dir_pin = 17;
  stepper[1].step_pin = 14;
  AX[1].STEPS_PER_MM = 1280;

  stepper[2].dir_pin = 11;
  stepper[2].step_pin = 10;
  AX[2].STEPS_PER_MM = 10666;

  stepper[3].dir_pin = 7;
  stepper[3].step_pin = 8;
  AX[3].STEPS_PER_MM = 1600;




  int i;
  for (i = 0; i < NUMBER_OF_AXIS; ++i) {
    pinMode(stepper[i].step_pin, OUTPUT);
    pinMode(stepper[i].dir_pin, OUTPUT);

  }
  pinMode(steppers_enable_pin, OUTPUT);
  digitalWrite(steppers_enable_pin, HIGH);
  pinMode(spindlepin, OUTPUT);
  digitalWrite(spindlepin, LOW);

  pinMode(heat1pin, OUTPUT);
  digitalWrite(heat1pin, LOW);

  pinMode(heat2pin, OUTPUT);
  digitalWrite(heat2pin, LOW);
}


void motor_enable() {
  digitalWrite(steppers_enable_pin, LOW);
}


void motor_disable() {
  digitalWrite(steppers_enable_pin, HIGH);
}


void Stop() {
  limit = 1;
}
void setup() {
  Serial.begin(BAUDRATE);

  pinMode(error_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(error_pin), Stop, FALLING);

  motor_setup();
  motor_enable();


  where();
  help();
  position(0, 0, 0, 0);
  feed_rate(DEFAULT_FEEDRATE);
  ready();
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (buffer_count < MAX_BUFFER - 1) buffer[buffer_count++] = c;
    if (c == '\n') {
      buffer[buffer_count] = 0;
      processCommand();
      Serial.println("ok");
      ready();
    }
  }
}
