#ifndef _ESP_TIMING_ENGINE_H_
#define _ESP_TIMING_ENGINE_H_

#define PIN_READ(x)           ((GPI & (1 << x)) != 0)
#define PIN_LOW(x)            (GPOC = (1 << x))
#define PIN_HIGH(x)           (GPOS = (1 << x))
#define SONAR_TRIG_PIN_LOW()  (GP16O &= ~1)
#define SONAR_TRIG_PIN_HIGH() (GP16O |= 1)

#define NUM_DEV  4
#ifdef USE_2_SERVOS
  #define NUM_DEV  5
#endif
///  Les variables des sous-programmes de la gestion des moteurs///
int16_t    speed_M1, speed_M2;        // La vitesse courante de la rotation des moteurs
int32_t    echo_start, echo_stop, echo_value=1000000;

typedef struct {
  int8_t         phase_or_dir;
  int32_t        a_period;
  int32_t        b_period;
  int32_t        c_period;
} device_header_t;

device_header_t  devices[NUM_DEV];

// La redéfinition des fonctions de l'interruption

#define xt_rsil(level) (__extension__({uint32_t state; __asm__ __volatile__("rsil %0," __STRINGIFY(level) : "=a" (state)); state;}))
#define xt_wsr_ps(state)  __asm__ __volatile__("wsr %0,ps; isync" :: "a" (state) : "memory")
#define te_write(count) __asm__ __volatile__("wsr %0,ccompare0; esync"::"a" (count) : "memory")

uint32_t te_getCycleCount()
{
    uint32_t ccount;
    __asm__ __volatile__("esync; rsr %0,ccount":"=a" (ccount));
    return ccount;
}

static volatile timercallback te_user_cb = NULL;

void ICACHE_RAM_ATTR te_isr_handler(void* para)
{
    if (te_user_cb) 
    {
      uint32_t savedPS = xt_rsil(15); // stop other interrupts
      te_user_cb();
      xt_wsr_ps(savedPS);   // start other interrupts
    }
}

void te_isr_init()
{
    ETS_CCOMPARE0_INTR_ATTACH(te_isr_handler, NULL);
}

void te_attachInterrupt(timercallback userFunc) {
    te_user_cb = userFunc;
    ETS_CCOMPARE0_ENABLE();
}

void te_detachInterrupt() {
    te_user_cb = NULL;
    ETS_CCOMPARE0_DISABLE();
}

// Output Motor+Servo+Sonar interrupt handle
void ICACHE_RAM_ATTR te_Processor(){ 
  
uint32_t cc;
  te_write(te_getCycleCount() + 675); // réglage de l'horloge pour 675 cycles en 10uS (340 pour 5 uS)
  PIN_LOW(MOTOR1_STEP_PIN);
  PIN_LOW(MOTOR2_STEP_PIN);
  for(int i=0; i < NUM_DEV; ++i)
  {
    if((--devices[i].a_period) == 0)
    {
      if(i == 0) // handle motor 1
      {
        if(devices[0].phase_or_dir < 0)
        {
          devices[0].a_period = devices[0].c_period;
        }
        else 
        {
          devices[0].a_period = devices[0].b_period;
          if(devices[0].phase_or_dir == !PIN_READ(MOTORS_DIR_PIN))
          {
            if(devices[0].phase_or_dir)
            {
              PIN_HIGH(MOTORS_DIR_PIN);             // DIR Motors for Motor1 (Forward)
            }
            else
            {
              PIN_LOW(MOTORS_DIR_PIN);              // DIR Motors for Motor1 (Revers)
            }
            cc = te_getCycleCount();
            while((te_getCycleCount() - cc) < PRE_DIR_STROBE);
          }
          PIN_HIGH(MOTOR1_STEP_PIN);
        }
      }
      else if(i == 1) // handle motor 2
      {
        if(devices[1].phase_or_dir < 0)
        {
          devices[1].a_period = devices[1].c_period;
        }
        else 
        {
          devices[1].a_period = devices[1].b_period;
          if(devices[1].phase_or_dir == PIN_READ(MOTORS_DIR_PIN))
          {
            if (devices[1].phase_or_dir)
            {
              PIN_LOW(MOTORS_DIR_PIN);                     // DIR Motors for Motor2 (Revers)
            }
            else
            {
              PIN_HIGH(MOTORS_DIR_PIN);                    // DIR Motors for Motor2 (Forward)
            }
            cc = te_getCycleCount();
            while((te_getCycleCount() - cc) < PRE_DIR_STROBE);
          }
          PIN_HIGH(MOTOR2_STEP_PIN);
        }
      }
      else if(i == 2) // handle sonar pulse
      {
        if(devices[2].phase_or_dir)
        {
          SONAR_TRIG_PIN_LOW();
          devices[2].phase_or_dir = 0;
          devices[2].a_period = devices[2].b_period - 1;
        }
        else
        {
          SONAR_TRIG_PIN_HIGH();
          devices[2].phase_or_dir = 1;
          devices[2].a_period = 1;    // handle 10us pulse or 1 pulse
        }
      }
      else if(i == 3) // handle servo 1 pulse
      {
        if(devices[3].phase_or_dir)
        {
          PIN_LOW(SERVO1_PIN);
          devices[3].phase_or_dir = 0;
          devices[3].a_period = devices[3].c_period - devices[3].b_period;
        }
        else
        {
          PIN_HIGH(SERVO1_PIN);
          devices[3].phase_or_dir = 1;
          devices[3].a_period = devices[3].b_period;    //positive pulse
        }
      }
#ifdef USE_2_SERVOS
      else if(i == 4) // handle servo 2
      {
        if(devices[4].phase_or_dir)
        {
          PIN_LOW(SERVO2_PIN);
          devices[4].phase_or_dir = 0;
          devices[4].a_period = devices[3].c_period - devices[3].b_period;
        }
        else
        {
          PIN_HIGH(SERVO2_PIN);
          devices[4].phase_or_dir = 1;
          devices[4].a_period = devices[3].b_period;    // positive pulse
        }
      }
#endif
    }
  }
}

// Sonar interrupt hanle (external interrupt) ??
void ICACHE_RAM_ATTR te_SonarEcho()
{
  if(PIN_READ(SONAR_ECHO_PIN))
  {
    echo_start = te_getCycleCount();
  }
  else
  {
    echo_stop = te_getCycleCount();
    echo_value = echo_stop - echo_start;
  }
}

void te_Start()
{
// Setup motor controler
  pinMode(MOTORS_ENABLE_PIN,OUTPUT);
  PIN_HIGH(MOTORS_ENABLE_PIN);    // driver pas actifs
  pinMode(MOTORS_DIR_PIN,OUTPUT);
  
// Setup motor 1 IO[0]
  pinMode(MOTOR1_STEP_PIN,OUTPUT);
  PIN_LOW(MOTOR1_STEP_PIN);
  devices[0].phase_or_dir = -1;  // -1 Le rien faire, 0 - le pas en arrière, 1 - le pas en avant
  devices[0].a_period = 1;
  devices[0].b_period = ZERO_SPEED;
  devices[0].c_period = ZERO_SPEED;
  
// Setup motor 2 IO[1]
  pinMode(MOTOR2_STEP_PIN,OUTPUT);
  PIN_LOW(MOTOR2_STEP_PIN);
  devices[1].phase_or_dir = -1;  // -1 ne rien faire, 1 = un pas en arrière, 0 = un pas en avant 
  devices[1].a_period = 1;
  devices[1].b_period = ZERO_SPEED;
  devices[1].c_period = ZERO_SPEED;
  
// Setup sonar device [2]
#ifdef SONAR_EN
  pinMode(SONAR_TRIG_PIN,OUTPUT);
  SONAR_TRIG_PIN_LOW();
  pinMode(SONAR_ECHO_PIN,INPUT);
  attachInterrupt(SONAR_ECHO_PIN, te_SonarEcho, CHANGE);
  devices[2].phase_or_dir = 0;
  devices[2].a_period = 1;
  devices[2].b_period = 4000;    // La signification maxima de la période de 40 millisecondes ou à la durée de la période du traitement 10 uS - 4000 périodes
#endif
// setup servo 1 devices[3]
  pinMode(SERVO1_PIN,OUTPUT);
  PIN_LOW(SERVO1_PIN);
  devices[3].phase_or_dir = 0;
  devices[3].a_period = 1;
  devices[3].b_period = SERVO_AUX_NEUTRO; 
  devices[3].c_period = 2000;  
#ifdef USE_2_SERVOS
  // setup servo 2 devices[4]
  pinMode(SERVO2_PIN,OUTPUT);
  PIN_LOW(SERVO2_PIN);
  devices[4].phase_or_dir = 0;
  devices[4].a_period = 1;
  devices[4].b_period = SERVO_AUX_NEUTRO; 
  devices[4].c_period = 2000; 
#endif
// Start process
  te_isr_init();
  te_attachInterrupt(te_Processor);
  te_write(te_getCycleCount() + 1500);
}

// Définition de la vitesse de la rotation des moteurs
// tspeed_Mx Peut être positif, ainsi que négatives (reverse)
void te_SetMotorsSpeed(int16_t t_speed_M1, int16_t t_speed_M2){
   
  int32_t   speed_1, speed_2;
  // Routine limitant le changement de la vitesse compte tenu de l'accélération maximale pour motor1
  if ((speed_M1 - t_speed_M1) > MAX_ACCEL)
    speed_M1 -= MAX_ACCEL;
  else if ((speed_M1 - t_speed_M1) < -MAX_ACCEL)
    speed_M1 += MAX_ACCEL;
  else
    speed_M1 = t_speed_M1;
    
  // Routine limitant le changement de la vitesse compte tenu de l'accélération maximale pour motor2
  if ((speed_M2 - t_speed_M2) > MAX_ACCEL)
    speed_M2 -= MAX_ACCEL;
  else if ((speed_M2 - t_speed_M2) < -MAX_ACCEL)
    speed_M2 += MAX_ACCEL;
  else
    speed_M2 = t_speed_M2;

  speed_1 = speed_M1 * K_MOTOR_SPEED;
  speed_2 = speed_M2 * K_MOTOR_SPEED;
  
  // Calcul de la période pour motor1
  if (speed_1 == 0)
  {
    devices[0].b_period = ZERO_SPEED;
    devices[0].phase_or_dir = -1;
  }
  else 
    if (speed_1 > 0)
    {
      devices[0].b_period = 6400000/speed_1;   // 6400000 = Vitess maximale avec speed_M1 (max. 500) * K_MOTOR_SPEED (max.12800)
      devices[0].phase_or_dir = 1;
    }
    else
    {
      devices[0].b_period = 6400000/-speed_1;
      devices[0].phase_or_dir = 0;
    }

  // Calcul de la période pour motor2
  if (speed_2 == 0)
  {
    devices[1].b_period = ZERO_SPEED;
    devices[1].phase_or_dir = -1;
  }
  else
    if (speed_2 > 0)
    {
      devices[1].b_period = 6400000/speed_2;
      devices[1].phase_or_dir = 1;
    }
    else 
    {
      devices[1].b_period = 6400000/-speed_2;
      devices[1].phase_or_dir = 0;
    }
}

void te_SetServo(int pwm, int num=0){
  
  if (num == 0)
    devices[3].b_period = pwm;
#ifdef USE_2_SERVOS
  if (num == 1)
    devices[4].b_period = pwm;
#endif
}
#endif // _ESP_TIMING_ENGINE_H_

