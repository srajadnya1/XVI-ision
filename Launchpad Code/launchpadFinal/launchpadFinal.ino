/*
 * integration.ino
 * Final sketch for SIXT33N Speech version
 *
 * EE16B Fall 2016
 * Emily Naviasky & Nathaniel Mailoa
 *
 * EE 16B Fall 2017
 * Andrew Blatner
 *
 */

/********************************************************/
/********************************************************/
/***                                                  ***/
/*** Constants and global variables from turning.ino. ***/
/***                                                  ***/
/********************************************************/
/********************************************************/

// Change pins here if you did not use the default pins!
#define LEFT_MOTOR                  P2_0
#define LEFT_ENCODER                P6_2
#define RIGHT_MOTOR                 P1_5
#define RIGHT_ENCODER               P6_3

#define SAMPLING_INTERVAL           100
int sample_lens[4] = {0};

// Operation modes
#define MODE_WATCH                  2
#define MODE_LISTEN                 0
#define MODE_DRIVE                  1
int timer_mode = MODE_WATCH; // ???

#define DRIVE_FAR                   0
#define DRIVE_LEFT                  1
#define DRIVE_CLOSE                 2
#define DRIVE_RIGHT                 3

#define JOLT_STEPS                  2

int stop_mode = 0;

boolean loop_mode = MODE_DRIVE;
boolean canRead = true;
int drive_mode = 0;

int step_num = 0;
volatile boolean do_loop = 0; // timer signal to increment timestep

typedef struct encoder {
  int pin;
  int pos;
  bool level;
  int avg;
} encoder_t;

encoder_t left_encoder = {LEFT_ENCODER, 0, LOW, 0};
encoder_t right_encoder = {RIGHT_ENCODER, 0, LOW, 0};

/*---------------------------*/
/*      CODE BLOCK CON1      */
/*      From turning.ino     */
/*---------------------------*/

float theta_left = 0.3114;
float theta_right =  0.3479;
float beta_left = -30.12;
float beta_right = -24.95;
float v_star = 78.4;

// PWM inputs to jolt the car straight
int left_jolt = 175;
int right_jolt = 175;

// Control gains
float f_left = 0.6;
float f_right = 0.65;

/*---------------------------*/
/*      CODE BLOCK CON2      */
/*      From turning.ino     */
/*---------------------------*/

float driveStraight_left(float delta) {
  return (1/theta_left) * (v_star - (f_left * delta) + beta_left);
}

float driveStraight_right(float delta) {
  return (1/theta_right) * (v_star + (f_right * delta) + beta_right);
}

/*---------------------------*/
/*      CODE BLOCK CON3      */
/*      From turning.ino     */
/*---------------------------*/

float delta_ss = 9;

/*---------------------------*/
/*      CODE BLOCK CON4      */
/*      From turning.ino     */
/*---------------------------*/

#define CAR_WIDTH                   15.0 // in cm
//#define TURN_RADIUS                 91 // in cm -  6 feet diameter
 #define TURN_RADIUS                 60 // in cm - 4 feet diameter

/*---------------------------*/
/*    PREPROGRAMMED PATH     */
/*---------------------------*/
int run_times[4] = {4000, 2000, 1000, 1600}; // length of commands roughly in ms
int drive_modes[4] = {DRIVE_FAR, DRIVE_LEFT, DRIVE_CLOSE, DRIVE_RIGHT}; // commands: [DRIVE_STRAIGHT, DRIVE_LEFT, DRIVE_RIGHT]

float delta_reference(int i) {
  // YOUR CODE HERE
  // Remember to divide the v* value you use in this function by 5 because of sampling interval differences!
  if (drive_mode == DRIVE_RIGHT) { // Return a NEGATIVE expression
    return (v_star * i * CAR_WIDTH)/ (5 * TURN_RADIUS);
  }
  else if (drive_mode == DRIVE_LEFT) { // Return a POSITIVE expression
    return -(v_star * i * CAR_WIDTH)/ (5 * TURN_RADIUS);
  }
  else { // DRIVE_STRAIGHT
    return -straight_correction(i);
  }
}
/*---------------------------*/
/*      CODE BLOCK CON5      */
/*      From turning.ino     */
/*---------------------------*/
#define INFINITY                    (3.4e+38)
#define STRAIGHT_RADIUS             5000

float straight_correction(int i) {
  // YOUR CODE HERE
  return (v_star * i * CAR_WIDTH)/ (5 * STRAIGHT_RADIUS);
}

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

/*********************************************************/
/*********************************************************/
/***                                                   ***/
/*** Constants and glboal variables from classify.ino. ***/
/***                                                   ***/
/*********************************************************/
/*********************************************************/

// Change pin here if you did not use the default pin!
#define MIC_INPUT                   P7_0

#define SIZE                        3200
#define SIZE_AFTER_FILTER           200
#define ADC_TIMER_MS                0.35

/*---------------------------*/
/*      CODE BLOCK PCA1      */
/*     From classify.ino     */
/*---------------------------*/

// Enveloping and threshold constants
#define SNIPPET_SIZE                  80
#define PRELENGTH                     5
#define THRESHOLD                     0.5

#define EUCLIDEAN_THRESHOLD         0.0285 // 0.03, 0.015
#define LOUDNESS_THRESHOLD          0.0085

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/


/*---------------------------*/
/*      CODE BLOCK PCA2      */
/*     From classify.ino     */
/*---------------------------*/

float pca_vec1[SNIPPET_SIZE] = {0.002610716115202973, 0.0013101231983110861, -0.004074336024386449, -6.762179305258575e-05, 0.01506912928789227, 0.013610319906556109, 0.021458981364077943, 0.01715039457202137, 0.02512974854337751, 0.036082748716670825, 0.06830309837797127, 0.08428276577143884, 0.1294007520681847, 0.1550982618223968, 0.17154999391051112, 0.19521244642508195, 0.20877939713228194, 0.21705240787120153, 0.22316999849748656, 0.21925066981691113, 0.20070044896641576, 0.21641827549206943, 0.19332761605431517, 0.1822361406749784, 0.172164254558059, 0.1536795708150913, 0.14093005945465176, 0.1327928227240489, 0.12192163483213349, 0.10882601149678153, 0.09854581433612127, 0.08272352147378033, 0.06416045768471526, 0.04284357214093866, 0.04240018408778098, 0.01796120405817779, 0.005896180916105841, -0.0067392427142765835, -0.01404654248069081, -0.02303679585765632, -0.03782571363455276, -0.032050155038378804, -0.03784320620275695, -0.042607681611283726, -0.0441576701638408, -0.04681569341624196, -0.050405704324475, -0.055177358883131994, -0.0549437625842507, -0.054806809606451054, -0.06295127790760693, -0.08069858384878893, -0.09906340291015796, -0.11619335336093853, -0.11851220421980706, -0.1273472958222547, -0.11894164900989905, -0.12901188329086605, -0.11984737166142025, -0.12211714526465459, -0.12216116382188737, -0.11702939589231669, -0.12279870929396175, -0.11900798482770188, -0.11535024491657404, -0.11735333595879566, -0.12690307382676488, -0.11551571958061527, -0.11588841071596138, -0.11785677316095777, -0.12303123651693085, -0.10854670014708588, -0.11107613877206748, -0.10830969354485348, -0.10417571488607398, -0.09577429221677967, -0.08804976301981103, -0.08763022425295587, -0.08740669598495875, -0.07890199019486671};
float pca_vec2[SNIPPET_SIZE] = {0.011019774395243228, 0.014025506785168063, -0.008464561335843876, 0.002050869959289603, 0.05746269485150435, 0.20265977991656664, 0.26636200988886904, 0.2508204643609422, 0.2605126502119718, 0.21838086904338072, 0.20014748642971114, 0.1735022512959809, 0.1559034878050691, 0.10753502205206691, 0.0489395631000819, 0.046220825618149455, 0.05197744465352673, 0.028698450974048127, 0.03288791183570228, 0.02243921119905324, -0.003793089892290752, 0.0001575970328120702, -0.008812863145461158, -0.005198693871714383, -0.004852648756633912, -0.00594617182067562, -0.01827967645265207, -0.03213867987130275, -0.05237150951889347, -0.066214766599233, -0.10263594791237059, -0.1284094204929188, -0.1479646458512306, -0.17662379990871135, -0.1784081356815684, -0.17932698129455854, -0.20095752416093615, -0.16852707040177556, -0.16904175660448556, -0.15207404395721344, -0.15939475027764313, -0.14728310788779883, -0.15061178546242227, -0.14637272659837658, -0.15836221417673585, -0.13449291632124358, -0.14907915363223953, -0.1421885096128533, -0.12264487487270455, -0.1312976663295193, -0.13128680653676034, -0.0858287162712564, -0.056862700486635165, 0.0007037977954304754, 0.013666861153966493, 0.04081269070180764, 0.0412384841630124, 0.05594672928330074, 0.04916684070660594, 0.062039824046918456, 0.059939106515766186, 0.05565550700245622, 0.06932887141623821, 0.06753722150276156, 0.06902164442719823, 0.055355200383115206, 0.07935727119730267, 0.05694165248539348, 0.07034082850892749, 0.06266686271858551, 0.06554803757145602, 0.06348720249920962, 0.06206476769101909, 0.06009047862157362, 0.061566075930261976, 0.044070155768612174, 0.030826619115095263, 0.033263996917066135, 0.03110891289656814, 0.012298403567872164};
float pca_vec3[SNIPPET_SIZE] = {0.04361647986410701, 0.010684776209872515, 0.048337440627079425, -0.007513839853639681, -0.06559240731062554, -0.10793301963776777, -0.14093935328706259, -0.15544961583227884, -0.1880364869018234, -0.21589283918832114, -0.22168136523862061, -0.24193423659020974, -0.21507232247508154, -0.22298287025789787, -0.1713511846569228, -0.08032792502450775, -0.019532817628668375, 0.06830995879375845, 0.1092197157151954, 0.14204170770265154, 0.1900154204650902, 0.2050058737858705, 0.21334595678234858, 0.20893217397422367, 0.19545260519895966, 0.16590188930413605, 0.1445230254574101, 0.11051333608913581, 0.07514486674016004, 0.03427635194017463, 0.016805718415001, -0.03017931623970431, -0.07310586648616935, -0.10953741374566332, -0.13657020705938608, -0.13569206314563645, -0.13688034195517937, -0.11011222337971033, -0.10400624174612205, -0.09326103311885556, -0.10525594887125374, -0.07039504481069539, -0.06830903856016393, -0.0989243731508798, -0.07009733859315434, -0.06545801549118503, -0.06393802916820673, -0.060929805206202295, -0.042681865238536815, -0.06460269643695206, -0.05674037430009942, -0.017455073238067238, -0.003844776292965173, -0.0006539522924342668, 0.031478165135111605, 0.04656344582792284, 0.033314974696563325, 0.022684276223535058, 0.046449759604667226, 0.03990284205487813, 0.05192538013674928, 0.05323728282302243, 0.07295015046854642, 0.050707154142620996, 0.04886443887112639, 0.05970725339624093, 0.06546667362613412, 0.06488676232358778, 0.06762802283542087, 0.06440504529407316, 0.06690339347381821, 0.08441915571808532, 0.09065712481588312, 0.10074561253563981, 0.10994431878449659, 0.09904019497087187, 0.10544950603082696, 0.09881138782549578, 0.11662530398019727, 0.0979763997499612};
float projected_mean_vec[3] = {0.005682863431992982, 0.02503304335380884, -0.027730467887647214};
float centroid1[3] = {0.06514159038870147, -0.0070202594165455715, -0.0029474471427854243}; // drive far - kiwi
float centroid2[3] = {-0.03513330978199226, -0.02662280541124067, 0.01747116241951199}; // left - reverse
float centroid3[3] = {-0.031710423859470024, 0.04363474018220452, 0.008837414465784649}; // drive close - sixteen
float centroid4[3] = {0.001702143252760839, -0.009991675354418268, -0.02336112974251123}; // right - coconut
float* centroids[4] = {
  (float *) &centroid1, (float *) &centroid2,
  (float *) &centroid3, (float *) &centroid4
};


/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

float result[SNIPPET_SIZE] = {0};
float proj1 = 0;
float proj2 = 0;
float proj3 = 0;

// Data array and index pointer
int re[SIZE] = {0};
volatile int re_pointer = 0;

/*---------------------------------------------------*/
/*---------------------------------------------------*/
/*---------------------------------------------------*/

/*---------------------------*/
/*       Norm functions      */
/*---------------------------*/

// Compute the L2 norm of (dim1, dim2) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        centroid: size-2 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm(float dim1, float dim2, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2));
}

// Compute the L2 norm of (dim1, dim2, dim3) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        dim3: 3rd dimension coordinate
//        centroid: size-3 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm3(float dim1, float dim2, float dim3, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2) + pow(dim3-centroid[2],2));
}

void setup(void) {
  Serial.begin(115200);
  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(LEFT_ENCODER, INPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RIGHT_ENCODER, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(MIC_INPUT, INPUT);

  for (int j = 0; j < 4; j++) {
    sample_lens[j] = run_times[j] / SAMPLING_INTERVAL;
  }

  write_pwm(0, 0);
  delay(2000); // Wait 2 seconds to put down car
  reset_blinker();
//   start_listen_mode();
}

void loop(void) {
  // for tmrw print time steps
//    Serial.println("start m");

  if (loop_mode == MODE_WATCH) {
    if (Serial.available() > 0 && canRead) {
        int best_index = 0;
        char msg = Serial.read();
        Serial.println("test message");
        if (msg == 's') {
            loop_mode = MODE_WATCH;
            stop_mode = 1;
        }
        else {
            digitalWrite(RED_LED, HIGH);
            Serial.println(msg);
            if (msg == 'l') {
                best_index = 1;
                Serial.println("left turn");
            }
            else if (msg == 'r') {
                best_index = 3;
                Serial.println("right turn");
            }
            loop_mode = MODE_DRIVE;
            canRead = false;
            drive_mode = best_index; // from 0-3, inclusive
            start_drive_mode();
        }
        
    }
  }
  check_encoders();
//   if (timer_mode == MODE_LISTEN && re_pointer == SIZE){
//     // Stop motor
//     write_pwm(0, 0);
//     digitalWrite(RED_LED, LOW);

//     // if enveloped data is above some preset value
//     if (envelope(re, result)) {

//       // Reset projection result variables declared above
//       proj1 = 0;
//       proj2 = 0;
//       proj3 = 0;

//       /*---------------------------*/
//       /*      CODE BLOCK PCA3      */
//       /*     From classify.ino     */
//       /*     with more changes     */
//       /*---------------------------*/

//       // Project 'result' onto the principal components
//       // YOUR CODE HERE
//       for (int i = 0; i < SNIPPET_SIZE; i++) {
//           proj1 += result[i] * pca_vec1[i];
//           proj2 += result[i] * pca_vec2[i];
//           proj3 += result[i] * pca_vec3[i];
//       }

//       // Demean the projection
//       proj1 -= projected_mean_vec[0];
//       proj2 -= projected_mean_vec[1];
//       proj3 -= projected_mean_vec[2];

//       // Classification
//       // Use the function 'l2_norm3' defined above
//       // ith centroid: 'centroids[i]'
//       float best_dist = 999999;
//       int best_index = -1;
//       // YOUR CODE HERE
//       for (int i = 0; i < 4; i++) {
//           float currNorm = l2_norm3(proj1, proj2, proj3, centroids[i]);
//           if (currNorm < best_dist) {
//               best_dist = currNorm;
//               best_index = i;
//           }
//       }


//       // Check against EUCLIDEAN_THRESHOLD and execute identified command
//       // YOUR CODE HERE
//       if (best_dist < EUCLIDEAN_THRESHOLD) {
//         drive_mode = best_index; // from 0-3, inclusive
//         start_drive_mode();
//       }

//       /*---------------------------*/
//       /*---------------------------*/
//       /*---------------------------*/
//     }

//     delay(2000);
//     re_pointer = 0; // start recording from beginning if we don't start driving
//   }


  if (loop_mode == MODE_DRIVE && do_loop) {
    if (step_num < JOLT_STEPS) {
      write_pwm(left_jolt, right_jolt);
    }
    else {

      // Save positions because _left_position and _right_position
      // can change in the middle of one loop.
      int left_position = left_encoder.pos;
      int right_position = right_encoder.pos;

      /*---------------------------*/
      /*      CODE BLOCK CON0      */
      /*---------------------------*/

      float delta = left_position - right_position + delta_ss;
      delta = delta + delta_reference(step_num) + straight_correction(step_num);

      // Drive straight using feedback
      // Compute the needed pwm values for each wheel using delta and v_star
      int left_cur_pwm = driveStraight_left(delta);
      int right_cur_pwm = driveStraight_right(delta);
      write_pwm(left_cur_pwm, right_cur_pwm);

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    }

    // Counter for how many times loop is executed since entering DRIVE MODE
    step_num++;

    if (step_num == sample_lens[drive_mode]) {
      // Completely stop and go back to listen MODE after 3 seconds
      canRead = true;
    //   start_listen_mode();
      loop_mode = MODE_WATCH;
    }

    do_loop = 0;
  }
}

// Enveloping function with thresholding and normalizing,
// returns snippet of interest (containing speech)
bool envelope(int* data, float* data_out) {
  int32_t avg = 0;
  float maximum = 0;
  int32_t total = 0;
  int block;

  // Apply enveloping filter while finding maximum value
  for (block = 0; block < SIZE_AFTER_FILTER; block++) {
    avg = 0;
    for (int j = 0; j < 16; j++) {
      avg += data[j+block*16];
    }
    avg = avg >> 4;
    data[block] = abs(data[block*16] - avg);
    for (int j = 1; j < 16; j++) {
      data[block] += abs(data[j+block*16] - avg);
    }
    if (data[block] > maximum) {
      maximum = data[block];
    }
  }

  // If not loud enough, return false
  if (maximum < LOUDNESS_THRESHOLD) {
    return false;
  }

  // Determine threshold
  float thres = THRESHOLD * maximum;

  // Figure out when interesting snippet starts and write to data_out
  block = PRELENGTH;
  while (data[block++] < thres);
  if (block > SIZE_AFTER_FILTER - SNIPPET_SIZE) {
    block = SIZE_AFTER_FILTER - SNIPPET_SIZE;
  }
  for (int j = 0; j < SNIPPET_SIZE; j++) {
    data_out[j] = data[block-PRELENGTH+j];
    total += data_out[j];
  }

  // Normalize data_out
  for (int j = 0; j < SNIPPET_SIZE; j++) {
    data_out[j] = data_out[j] / total;
  }

  return true;
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void write_pwm(int pwm_left, int pwm_right) {
  analogWrite(LEFT_MOTOR, (int) min(max(0, pwm_left), 255));
  analogWrite(RIGHT_MOTOR, (int) min(max(0, pwm_right), 255));
}

void reset_blinker(void) {
  digitalWrite(RED_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, LOW);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(GREEN_LED, LOW);
}

// void start_listen_mode(void) {
//   re_pointer = 0;
//   write_pwm(0, 0);
//   delay(3000); // 3 seconds buffer for mic cap settling
//   timer_mode = MODE_LISTEN;
//   setTimer(MODE_LISTEN);
// }

void start_drive_mode(void) {
  timer_mode = MODE_DRIVE;
  step_num = 0;
  left_encoder.pos = 0;
  right_encoder.pos = 0;
  setTimer(MODE_DRIVE);
}

/*---------------------------*/
/*    Interrupt functions    */
/*---------------------------*/

#define AVG_DECAY_RATE              0.3
#define LOW_THRESH                  ((int) (0.1*4096))
#define HIGH_THRESH                 ((int) (0.4*4096))

void check_encoder(encoder_t* enc) {
  int new_val = analogRead(enc->pin);
  enc->avg = (int) (AVG_DECAY_RATE*enc->avg + (1 - AVG_DECAY_RATE)*new_val);
  if ((enc->level == LOW && HIGH_THRESH < enc->avg) ||
      (enc->level == HIGH && enc->avg < LOW_THRESH)) {
    enc->pos++;
    enc->level = !enc->level;
  }
}

void check_encoders(void) {
  check_encoder(&left_encoder);
  check_encoder(&right_encoder);
}

// Set timer for timestep; use B0 to free up all other PWM ports
void setTimer(boolean mode) {
  if (mode == MODE_LISTEN) {
    // Set the timer based on 25MHz clock
    TB0CCR0 = (unsigned int) (25000*ADC_TIMER_MS);
    TB0CCTL0 = CCIE;
    __bis_SR_register(GIE);
    TB0CTL = TASSEL_2 + MC_1 + TACLR + ID_0;
  }
  else if (mode == MODE_DRIVE) {
    TB0CCR0 = (unsigned int) (32.768*SAMPLING_INTERVAL); // set the timer based on 32kHz clock
    TB0CCTL0 = CCIE; // enable interrupts for Timer B
    __bis_SR_register(GIE);
    TB0CTL = TASSEL_1 + MC_1 + TACLR + ID_0;
  }
  timer_mode = mode;
}

// ISR for timestep
#pragma vector=TIMER0_B0_VECTOR    // Timer B ISR
__interrupt void Timer0_B0_ISR(void) {
  if (timer_mode == MODE_LISTEN) {
    if (re_pointer < SIZE) {
//      digitalWrite(RED_LED, HIGH);
      re[re_pointer] = (analogRead(MIC_INPUT) >> 4) - 128;
      re_pointer += 1;
    }
  }
  else if (timer_mode == MODE_DRIVE) {
    do_loop = 1;
  }
}
