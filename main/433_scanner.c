
#include "433_scanner.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include "soc/timer_group_struct.h"
#include "include/output.h"

#include "nvs_flash.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "esp_system.h"
#include "esp_types.h"
#include "sdkconfig.h"

#define INP_PIN               21
#define DEBUG_LED0            22
#define DEBUG_LED1            23
#define RF_LED                18
#define PKT_LED               19
#define ADVANCED_OUTPUT       1
#define REQUIRE_DOUBLE        1
#define TAG                   "RF-RECEIVER"

#define TOLERANCE             20    // Tolerance percentage
#define SEPERATION_LIMIT      4300
#define MIN_DURATION          25000
#define TIMEOUT_NOCHANGE      15000
#define MIN_CHANGECOUNT       10    // Minimum state changes to process

#define TIMER_INTERVAL        1
#define TIMER_DIVIDER         16    // Hardware timer clock divider
#define TIMER_DIVIDER1MS      8000  // 10000Hz 
#define TIMER_DIVIDER1mS      8     // 10000000Hz 10MHz
#define TIMER_FINE_ADJ        (1.4*(TIMER_BASE_CLK / TIMER_DIVIDER)/1000000)    /*!< used to compensate alarm value (7)*/
#define TIMERMS_FINE_ADJ      (1.4*(TIMER_BASE_CLK / TIMER_DIVIDER1MS)/1000000) /*!< used to compensate alarm value */
#define TIMERmS_FINE_ADJ      (1.4*(TIMER_BASE_CLK / TIMER_DIVIDER1mS)/1000000) /*!< used to compensate alarm value (14) */
#define TIMERVALUE(x)         (x*5000000 - TIMER_FINE_ADJ)
#define TIMERVALUEuS(x)       (x*10) - TIMERMS_FINE_ADJ
#define TIMERVALUEmS(x)       (x*10000 - TIMERmS_FINE_ADJ)

#define COLOR_PRINT_BLACK     "30"
#define COLOR_PRINT_RED       "31"
#define COLOR_PRINT_GREEN     "32"
#define COLOR_PRINT_BROWN     "33"
#define COLOR_PRINT_BLUE      "34"
#define COLOR_PRINT_PURPLE    "35"
#define COLOR_PRINT_CYAN      "36"
#define color_printf(COLOR,format, ... ) { printf("\033[0;" COLOR "m" format "\033[0m", ##__VA_ARGS__); }

typedef struct
{
  unsigned long time;
  unsigned long startTime;
  unsigned long lastTime;
  unsigned int  length;
  unsigned int  interval;
  unsigned int  *timings;
} wireless_event_t;
xQueueHandle event_queue;

portMUX_TYPE microsMux = portMUX_INITIALIZER_UNLOCKED;
unsigned long IRAM_ATTR micros ()
{
  static unsigned long lccount = 0;
  static unsigned long overflow = 0;
  unsigned long ccount;

  portENTER_CRITICAL_ISR (&microsMux);

  __asm__ __volatile__ ("rsr     %0, ccount":"=a" (ccount));
  if (ccount < lccount)
  {
    overflow += UINT32_MAX / CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ;
  }
  lccount = ccount;

  portEXIT_CRITICAL_ISR (&microsMux);

  return overflow + (ccount / CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ);
}

// Debug
bool debug_led0 = 0;
bool debug_led1 = 0;

// ---
static unsigned int timings[RCSWITCH_MAX_CHANGES];
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
IRAM_ATTR void microsCallback(void *pArg)
{
  int timer_idx = (int) pArg;

  static unsigned int     statusLED = 0;
  static unsigned int   changeCount = 0;
  static unsigned long    startTime = 0;
  static unsigned long     lastTime = 0;
  static bool          lastInpState = 0;
  static bool                 ledOn = false;

  portENTER_CRITICAL(&timerMux);
  bool inpState = gpio_get_level (INP_PIN);
  portEXIT_CRITICAL(&timerMux);

  // Get the current time
  const unsigned long time = micros();

  // Check if we need to process this run as a pulse change.
  // If the input level changed, then we need to do checks.
  unsigned char checkState = 0;
  if ( lastInpState != inpState )
  {
    ledOn ^= true;
    gpio_set_level (RF_LED, ledOn);

    checkState = 1;
  }
  // If our startTime is greater than zero then we may have
  // a series of pulses followed by a very long low signal.
  else if ( startTime )
  {
    // Force a check if we have not received a signal change for
    // sometime but have data waiting to be processed.
    if ( (time - lastTime) > TIMEOUT_NOCHANGE )
    {
      checkState = 2;
    }
  }
  //else if ( (time - lastTime) > SEPERATION_LIMIT )
  //{
  //  lastTime = time;
  //}

  if ( checkState )
  {
    // Calculate pulse length
    const unsigned int duration = time - lastTime;
 
    // Save this pulse
    timings[changeCount++] = duration;

    // Period of time between two 'duration's
    const unsigned long interval = time - startTime;

    // A long stretch without signal level change occurred. This could
    // be the start/end of a transmission.
    if ( duration > SEPERATION_LIMIT )
    {
      // Debug helper
      //gpio_set_level(DEBUG_LED0, (debug_led0 = !debug_led0));

      if ( startTime )
      {
        // Debug helper
        //gpio_set_level(DEBUG_LED1, (debug_led1 = !debug_led1));

        // If we have a previous gap and this gap was at least
        // 'x' length of time ago, then create an alert.
        if ( (interval > MIN_DURATION) && changeCount > MIN_CHANGECOUNT )
        {
          // Debug helper
          //gpio_set_level(DEBUG_LED1, (debug_led1 = !debug_led1));

          wireless_event_t evt;
          evt.time      = time;
          evt.startTime = startTime;
          evt.lastTime  = lastTime;
          evt.interval  = interval;
          evt.length    = changeCount - 1;
          evt.timings   = pvPortMalloc(evt.length * sizeof(int));
          memcpy(evt.timings, timings, sizeof(int) * evt.length);

          /* Now just send the event data back to the main program task */
          xQueueSendFromISR(event_queue, &evt, NULL);

          gpio_set_level(PKT_LED, 1);
          statusLED = 500;
        }

        // Reset changeCount to zero
        changeCount = 0;

        // Assume that this could also be the start of the next packet.
        // Save this pulse
        timings[changeCount++] = duration;
      }

      // Set the start time
      if ( checkState == 1 )
      {
        startTime = lastTime;
      }
      else if ( checkState == 2 )
      {
        startTime = 0;   // Reset our start time
        changeCount = 0; // Reset our history
      }
    }

    // Detect overflow
    if ( changeCount >= RCSWITCH_MAX_CHANGES )
    {
      changeCount = 0;
    }

    // Save our Input state
    lastInpState = inpState;

    // Save the time this pulse ended (beginning of next pulse)
    lastTime = time;
  }
  else
  {
    if ( statusLED )
    {
      if ( --statusLED == 0 )
      {
        gpio_set_level(PKT_LED, 0);
      }
    }
  }

  TIMERG1.hw_timer[timer_idx].update = 1;
  TIMERG1.int_clr_timers.t1 = 1; //isr ack
  TIMERG1.hw_timer[timer_idx].config.alarm_en = 1;
}

#undef MIN
#undef MAX
#define MAX(a,b) (((a)>(b)) ? (a) : (b))
#define MIN(a,b) (((a)<(b)) ? (a) : (b))

#define BTST(a,b) ((a) & (b))
#define BSET(a,b) ((a) |= (b))
#define BCLR(a,b) ((a) &= ~(b))
#define BCHG(a,b) ((a) = (a) ^ (b))

#define EML(a) (((a) * (100 - TOLERANCE)) / 100)
#define EMH(a) (((a) * (100 + TOLERANCE)) / 100)
#define PULSE_WIDTH(p,w) ((p) > EML(w) && p < EMH(w))

// Correct Manchester Encoding
#define DECODE_MANCHESTER(x,y) (((x) > (y)) ? 1 : 0)
// HE Manchester Encoding
#define DECODE_HE_MANCHESTER(x,y) (((x) > (y)) ? 1 : ((y) > (x)) ? 0 : (x) ? 2 : 3)

#define MAX_PULSELEN    650
#define PROTO_HOMEEASY  1

unsigned char decode_bit(unsigned int a, unsigned int b)
{
  if ( PULSE_WIDTH(a, 310) && PULSE_WIDTH(b, 1250) )
  {
    return 1;
  }
  else if ( PULSE_WIDTH(a, 310) && PULSE_WIDTH(b, 220) )
  {
    return 0;
  }
  else
  {
    return 2;
  }
}

bool _check_homeEasy(wireless_event_t evt)
{
  bool retVal = false;

  unsigned int  bc=0;
  unsigned char bits[72];
  unsigned long deviceID = 0;
  unsigned int  data     = 0;

  unsigned char state;
  unsigned int  module;
  unsigned int  group;
  unsigned int  level    = 0;

  unsigned int  protocol = 0;
  unsigned int  pulseLen = 0;

  // Need to analyse the counts rather than just basing the timings off
  // one device...

  // 320uS / 2550uS preamble (from my measurements)
  if ( PULSE_WIDTH(evt.timings[1], 320) && PULSE_WIDTH(evt.timings[2], 2550) )
  {
    // Work out which protocol is being used...
    for ( unsigned int i=3; i < evt.length - 2; i += 2 )
    {
      // 
      // Only mean pulses that have a significant difference
      // between the high and low.
      //
      if ( MAX(evt.timings[i], evt.timings[i+1]) > MAX_PULSELEN )
      {
        pulseLen += MIN(evt.timings[i], evt.timings[i+1]);
        bc++;
      }

    }
    pulseLen = pulseLen / bc;

    // Try to determine our protocol from the pulse length
    if ( PULSE_WIDTH(pulseLen, 310) )
    {
      protocol = PROTO_HOMEEASY;
    }
    else
    {
      return false;
    }
 
    // Iterate through the state changes, skipping the
    // pause and preamble.
    for ( unsigned int i=3, bc=0; i < evt.length - 2; i += 2, bc++ )
    {
      unsigned char bit = decode_bit(evt.timings[i], evt.timings[i+1]);
      if ( bit < 2 )
      {
        bits[bc] = bit;
      }
      else
      {
        return false;
      }
    }

#ifdef __DEBUG__
    for ( unsigned int i=0, bc=1; i<70; i += 2, bc++)
    {
      printf("%d", DECODE_HE_MANCHESTER(bits[i], bits[i+1]));
      if ( !(bc % 4) )
        printf(" ");
    }
    printf("\n");
#endif

    // Get the device identification
    for ( unsigned int i=0, bc=1; i<48; i += 2, bc++ )
    {
      unsigned char bit = DECODE_MANCHESTER(bits[i], bits[i+1]);
      deviceID = (deviceID << 1) | (bit & 0x01);

#ifdef __DEBUG__
      printf("%d", bit);
      if ( !(bc % 4) )
        printf(" ");
    }
    printf("\n");
#else
  }
#endif

    // Get the data (on/off/target)
    for ( unsigned int i=48, bc=1; i<70; i += 2, bc++ )
    {
      unsigned char bit = DECODE_HE_MANCHESTER(bits[i], bits[i+1]);
      data = (data << 1) | (bit & 0x01);

#ifdef __DEBUG__
      printf("%d", bit);
      if ( !(bc % 4) )
        printf(" ");
    }
    printf("\n");
#else
    }
#endif

    // 27th bit => on/off or (dimmer)
    state = DECODE_HE_MANCHESTER(bits[54], bits[55]);

    // 28th, 29th & 30th => lighting modules
    module = (data & 0x70) >> 4;

    // 25th & 31st => group
    group = ((data & 0x400) >> 9) | ((data & 0x08) >> 3);

    // 32nd, 33rd & 34th => dim level (0 - 7)
    if ( state == 0x3 )
    {
      level = (data &0x07);
    }
 
    printf("Nexa / HomeEasy: deviceID: %ld, group: %d, module: %d, Function: %s, Dim Level: %d\n",
        deviceID,group,module,(state == 1 ? "On" : (state == 0 ? "Off" : "Dim")), level);

    retVal = true;
  }

  return retVal;
}

void _print_usecs(wireless_event_t evt)
{
  //color_printf(COLOR_PRINT_BLUE, "time = %ld, startTime = %ld, lastTime = %ld, interval = %d, length = %d\n",
  //    evt.time, evt.startTime, evt.lastTime, evt.interval, evt.length);
  //color_printf(COLOR_PRINT_BLUE, "Pulses = %d\n", evt.length);
  //printf("time = %ld, startTime = %ld, lastTime = %ld, interval = %d\n", evt.time, evt.startTime, evt.lastTime, evt.interval);
  //printf("Pulses = %d\n", evt.length);

  //"color_printf(COLOR_PRINT_BLUE, "(uSec)=");
  //printf("(uSec)=");
  for ( unsigned int i=0; i < evt.length; i++ )
  {
    if (i)
      printf(",");
    //color_printf(COLOR_PRINT_RED, "%d", evt.timings[i]);
    printf("%d", evt.timings[i]);
  }
  printf(";\n");
}

void _scan_pulses(wireless_event_t evt)
{
  unsigned long minPulseLen = 1000, maxPulseLen = 0, totPulseLen = 0, pulses = 0;
  unsigned long aPulseLen = 0, lPulseLen = 0, hPulseLen = 0;

  // Get min/max/count of pulses
  for ( unsigned int i=3; i < evt.length - 1; i++ )
  {
    if ( evt.timings[i] > maxPulseLen )
    {
      maxPulseLen = evt.timings[i];
    }
    else if ( evt.timings[i] < minPulseLen )
    {
      minPulseLen = evt.timings[i];
    }

    totPulseLen += evt.timings[i];

    pulses++;
  }
  aPulseLen = totPulseLen / pulses;

  if ( aPulseLen < 900 )
  printf("pulses: %ld, minPulseLen: %ld, maxPulseLen: %ld, aPulseLen: %ld, lPulseLen: %ld, hPulseLen: %ld\n",
      pulses, minPulseLen, maxPulseLen, aPulseLen, lPulseLen, hPulseLen);

  // Scan the event, skipping the leading space and any
  // preamble that might be there.
  for ( unsigned int i=3; i < evt.length - 2; i += 2 )
  {
    if ( MAX(evt.timings[i], evt.timings[i+1]) > MAX_PULSELEN )
    {
      //pulseLen += MIN(evt.timings[i], evt.timings[i+1]);
    }
  }
}

void scanner (void *pvParameter)
{
  wireless_event_t evt;

  while (1)
  {
    if ( xQueueReceive(event_queue, &evt, 10) == pdTRUE )
    {
      // Scan pulses to glean information
      _scan_pulses(evt);

      // Known decode handlers
      if ( _check_homeEasy(evt) )
      {
        _print_usecs(evt);
      }

      // Print pulses for debugging
      //_print_usecs(evt);

      vPortFree(evt.timings);
    }
  }
}

static void init_usec_timer()
{
  timer_config_t config;
  config.alarm_en     = 1;
  config.auto_reload  = TIMER_AUTORELOAD_DIS;
  config.counter_dir  = TIMER_COUNT_UP;
  config.divider      = TIMER_DIVIDER;
  config.intr_type    = TIMER_INTR_LEVEL;
  config.counter_en   = TIMER_PAUSE;

  /*Configure timer 1µS*/
  config.auto_reload = TIMER_AUTORELOAD_EN;
  config.divider = TIMER_DIVIDER1mS;
  ESP_ERROR_CHECK(timer_init(TIMER_GROUP_1, TIMER_1, &config));
  ESP_ERROR_CHECK(timer_pause(TIMER_GROUP_1, TIMER_1));
  ESP_ERROR_CHECK(timer_isr_register(TIMER_GROUP_1, TIMER_1, microsCallback, (void*) TIMER_1, 0, NULL));

  /* start 1µS timer*/
  ESP_ERROR_CHECK(timer_set_counter_value(TIMER_GROUP_1, TIMER_1, 0x00000000ULL));
  ESP_ERROR_CHECK(timer_set_alarm_value(TIMER_GROUP_1, TIMER_1,TIMERVALUEuS(10))); // 10 us timer
  ESP_ERROR_CHECK(timer_enable_intr(TIMER_GROUP_1, TIMER_1));
  ESP_ERROR_CHECK(timer_set_alarm(TIMER_GROUP_1, TIMER_1,TIMER_ALARM_EN));
  ESP_ERROR_CHECK(timer_start(TIMER_GROUP_1, TIMER_1));
}

void app_main ()
{
  nvs_flash_init ();

  // Configure the data input
  gpio_config_t inp_pin_config = {
    .intr_type                = GPIO_INTR_ANYEDGE,
    .mode                     = GPIO_MODE_INPUT,
    .pin_bit_mask             = (1 << INP_PIN),
    .pull_up_en               = GPIO_PULLUP_DISABLE,
    .pull_down_en             = GPIO_PULLDOWN_DISABLE
  };
  gpio_config (&inp_pin_config);

  // Output/LED Pins
  gpio_config_t out_pin_config;
  out_pin_config.intr_type    = GPIO_INTR_DISABLE;
  out_pin_config.mode         = GPIO_MODE_OUTPUT;
  out_pin_config.pin_bit_mask = ((1ULL << DEBUG_LED0) | (1ULL << DEBUG_LED1) | (1ULL << RF_LED) | (1ULL << PKT_LED));
  out_pin_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
  out_pin_config.pull_up_en   = GPIO_PULLUP_DISABLE;
  gpio_config (&out_pin_config);

  // Create an event queue
  event_queue = xQueueCreate(10, sizeof(wireless_event_t));

  // Initialise our timer queue
  init_usec_timer();

  xTaskCreate (&scanner, "433_scanner", 2048, NULL, 5, NULL);
}
