/*ECG simulator (Chicken Heart) v1.6a
  Requires terminal emulator to enter variables
  Outputs ECG templates ECG1 and ECG3 (PVC) with isoelectric 
  line between templates on pin A3
  ECG baseline is at 1.65V. ECG peak-to-peak is +/- 1V (.65V - 2.65V)
  Isoelectric length is controlled by heart rate 
  Up to a total of 50% 60-Hz plus 50% EMG plus 50% 0.25 Hz noise can be added
  Outputs pulse on D13 (LED1) with a width equal to QRS duration and flashes LED1
  Willis Tompkins - 2/25/21
  
  Modified by Royal Oakes - 1/13/2022
*/
#include "mbed.h"

AnalogOut ECG_output(A3);
DigitalOut QRS_led(LED1);
Ticker samp_rate;
Serial pc(USBTX, USBRX);
Serial device(D1, D0);

typedef union _data { // Custom union for sending data to device.
    float f;
    char s[4];
} myData;

myData d;

#define BUFFER_SIZE1 (163)
#define BUFFER_SIZE3 (163)
#define ECG1_QRS_onset (46)
#define ECG1_QRS_offset (65)
#define ECG3_QRS_onset (6)
#define ECG3_QRS_offset (61)
#define pi (3.14159)

//ECG templates computed from original ECG1 & ECG3 (PVC) templates 
//   floating point numbers = (Orig_data_sample + 4095) / 8192
//   This normalizes original data template from range of +/-4095 to 
//   range of 0.0-1.0 for DAC AnalogOut command.
//   Value 0.0 represents 0V--most negative ECG voltage.
//   Value 0.5 represents about 1.6V--ECG baseline.
//   Value 1.0 represents about 3.2V--most positive ECG voltage.  

float  ECG1[BUFFER_SIZE1]={0.500,0.501,0.500,0.500,0.499,0.501,0.502,
0.500,0.500,0.509,0.524,0.538,0.547,0.562,0.566,0.566,0.566,0.557,0.552,
0.543,0.533,0.524,0.519,0.509,0.505,0.505,0.500,0.500,0.505,0.505,0.505,
0.505,0.505,0.509,0.509,0.509,0.514,0.514,0.514,0.514,0.514,0.514,0.514,
0.514,0.514,0.514,0.528,0.552,0.595,0.657,0.742,0.842,0.932,0.990,0.999,
0.956,0.866,0.752,0.628,0.524,0.304,0.091,0.001,0.001,0.091,0.197,0.322,
0.411,0.482,0.500,0.505,0.509,0.505,0.505,0.505,0.505,0.500,0.500,0.500,
0.500,0.500,0.500,0.500,0.500,0.500,0.500,0.500,0.500,0.505,0.505,0.509,
0.509,0.514,0.514,0.519,0.519,0.524,0.528,0.533,0.538,0.538,0.543,0.547,
0.547,0.552,0.552,0.557,0.557,0.557,0.562,0.566,0.576,0.585,0.595,0.604,
0.614,0.628,0.638,0.647,0.661,0.676,0.685,0.700,0.709,0.723,0.733,0.747,
0.757,0.761,0.766,0.771,0.771,0.776,0.776,0.771,0.771,0.766,0.766,0.757,
0.747,0.738,0.723,0.704,0.690,0.671,0.657,0.638,0.623,0.609,0.595,0.581,
0.566,0.557,0.547,0.538,0.533,0.524,0.514,0.509,0.505,0.505,0.500,0.500};

//ECG3 is a PVC beat template
float ECG3[BUFFER_SIZE3]={0.500,0.500,0.500,0.500,0.499,0.501,0.502,0.500,
0.500,0.495,0.489,0.484,0.478,0.467,0.451,0.434,0.418,0.407,0.396,0.385,
0.374,0.363,0.352,0.341,0.319,0.297,0.275,0.253,0.231,0.209,0.182,0.154,
0.132,0.110,0.089,0.067,0.045,0.028,0.017,0.006,0.001,0.001,0.001,0.006,
0.012,0.023,0.039,0.061,0.083,0.116,0.149,0.193,0.237,0.281,0.319,0.352,
0.385,0.418,0.451,0.484,0.500,0.527,0.561,0.595,0.636,0.664,0.684,0.698,
0.705,0.718,0.725,0.739,0.746,0.752,0.752,0.759,0.759,0.766,0.766,0.773,
0.773,0.780,0.787,0.800,0.800,0.807,0.807,0.814,0.821,0.834,0.841,0.841,
0.848,0.855,0.862,0.869,0.875,0.882,0.889,0.896,0.903,0.909,0.923,0.930,
0.937,0.944,0.950,0.964,0.971,0.971,0.978,0.978,0.985,0.991,0.998,0.998,
0.998,0.998,0.991,0.985,0.978,0.971,0.957,0.944,0.937,0.923,0.909,0.896,
0.875,0.855,0.834,0.821,0.807,0.787,0.793,0.752,0.739,0.718,0.698,0.677,
0.657,0.643,0.623,0.609,0.595,0.582,0.568,0.561,0.554,0.541,0.527,0.520,
0.513,0.513,0.507,0.507,0.500,0.500,0.500,0.500,0.500,0.500,0.500};


int hr = 60; //Heart rate in beats per minute (range limited to 40-120 BPM)
int i = 0;
int j = 0;
int k = 0;
int p = 1;
static int m = 0;
static int n = 0;
int ab_beat_freq = 1;  // # of normal beats before each abnormal beat.
                       // setting of 1 is normal ECG
int iso_pts;
int iso_pts3;
int base_pts;
char beat_type = 'V';   //N = normal ECG; V = periodic PVC with full 
                        //compensatory pause; S = periodic skipped beat
float ampl_lf = 0.0;    // 0.25 Hz noise - percent of peak-to-peak ECG amplitude
float ampl_60Hz = 0.0;  // 60-Hz noise - percent of peak-to-peak ECG amplitude
float ampl_rand = 0.0;  //EMG (random noise) - percent of peak-to-peak ECG amplitude
char input1;
int numb_beats;
float noise_level;

// Function prototypes
void kbd_IO();
void samp_out();
void plot_ECG1(int i);
void plot_ECG3(int k);
void plot_baseline (int j);
void plot_QRS1_pulse(int i);
void plot_QRS3_pulse(int k);
int get_numb_beats(void);
float get_percent_noise(void);
float noise(void);

int main() {
    device.baud(115200);

// Program needs terminal emulator for keyboard input
    kbd_IO();

//Start 360 sps clock ticking   
    samp_rate.attach_us(&samp_out,2778);  //Output 360 samples/s (2778 us/samp)
    
    while(1) {      
    input1 = pc.getc();
    if (input1 == 27) {kbd_IO();}  // ESC key (27 ASCII) restarts program
    }
}
//Interrupt Service Routine
//Enters here 360 times per second (every 2278 microseconds
void samp_out() {
    //Plots normal beat (ECG1)
    if (i < BUFFER_SIZE1) {
        plot_ECG1(i);         //Plots normal (ECG1) template
        plot_QRS1_pulse(i);   //Plots pulse for each QRS complex and flashes LED1
        i++;
        return;
    }
    // Plots PVC (ECG3)
    if ((beat_type == 'V') && (p == ab_beat_freq)) {
        if (k < BUFFER_SIZE3) {
            plot_ECG3(k);         //Plots PVC (ECG3) template
            plot_QRS3_pulse(k);   //Plots pulse for each PVC and flashes LED1
            k++;
            return;
        }
    }
    //Sets isolectric length according to cardiac rhythm chosen
    if (beat_type == 'N') {base_pts = iso_pts;}
    if ((p == ab_beat_freq) && (beat_type == 'V')) {base_pts = iso_pts3;}
    if ((p == ab_beat_freq) && (beat_type == 'S')) {base_pts = 2*iso_pts + BUFFER_SIZE1;}

    if (j < base_pts) {
        plot_baseline(j);     //Plots the isoelectric line between beats
        j++;            
        return;
    }

    if (p == ab_beat_freq){p = 1;}  //Resets the count of normal beats to one
    else {p++;}       //Increments the count of normal beats before next beat
  
    i = 0;
    j = 0;
    k = 0;
    base_pts = iso_pts;
}
//Plots one sample point of normal (ECG1) template
void plot_ECG1(int i) {
    float temp = 0.5f + 0.3f*((ECG1[i]- 0.5f) + noise());
    ECG_output = temp;
    d.f = temp;
    for (int l = 0; l < 4; l++)
        device.putc(d.s[l]);
    device.putc('\n');
    return;
}
//Plots one sample point of PVC (ECG3) template
void plot_ECG3(int k) {
    float temp = 0.5f + 0.3f*((ECG3[k]- 0.5f) + noise());
    ECG_output = temp;
    d.f = temp;
    for (int l = 0; l < 4; l++)
        device.putc(d.s[l]);
    device.putc('\n');
    return;
}
//Plots one sample point of isoelectric line
void plot_baseline(int j) {
    float temp = 0.5f + 0.3f * noise();
    ECG_output = temp;
    d.f = temp;
    for (int l = 0; l < 4; l++)
        device.putc(d.s[l]);
    device.putc('\n');
    return;
    }

float noise(void) {
    if (++m > 5) {m = 0;}
    if (++n > 1439) {n = 0;}
    return (ampl_rand)*((float)rand()/RAND_MAX - 0.5f) +     //EMG (random) noise
    (ampl_60Hz*(sin(pi/6.0 + m*pi/3.0))) +  //60-Hz noise
    (ampl_lf * (sin(n*pi/720.0)));            //0.25-Hz noise
}
    
//Plots the onset and offset of the QRS duration to produce a pulse
//proportional to QRS duration on PB3 and flashes LED1
void plot_QRS1_pulse(int i) {
    if (i == ECG1_QRS_onset) {QRS_led = 1; return;}
    if (i == ECG1_QRS_offset) {QRS_led = 0; return;}   
    }
void plot_QRS3_pulse(int k) {
    if (k == ECG3_QRS_onset) {QRS_led = 1; return;}
    if (k == ECG3_QRS_offset) {QRS_led = 0; return;}   
    }
int get_numb_beats(void) {
    int numb_beats;
        pc.printf("\n\rEnter number of normal beats prior to each abnormal beat plus RETURN/ENTER: ");
        scanf("%d",&numb_beats);
        pc.printf("%d", numb_beats);
        return numb_beats;
}
float get_percent_noise(void) {
    float noise_level;
    pc.printf("Enter percent noise (0-50) plus RETURN/ENTER: ");
    scanf("%f",&noise_level);
        if (noise_level < 0.0f || noise_level > 50.0f) {
            pc.printf("\n\rError - noise set to zero. Restart program to reset noise level.\n\r");
            return 0.0f;
        }
        pc.printf("Percent: %2.0f\n\r", noise_level);
        return noise_level;
}

void kbd_IO() {
            pc.printf("\n\r\nEnter ECG mode: N = normal, V = PVCs, S = Skipped beats: ");
        input1 = pc.putc(pc.getc());
        
        switch (input1) {
            case 'N': case 'n': beat_type = 'N'; pc.printf(" - Normal ECG");
            break; 
            case 'V': case 'v': beat_type = 'V'; pc.printf(" - PVCs");
            ab_beat_freq = get_numb_beats();
            break;
            case 'S': case 's': beat_type = 'S'; pc.printf(" - Skipped beats");
            ab_beat_freq = get_numb_beats();
            break;
        default:
            pc.printf("\n\rError - Mode set to Normal\n");
            input1 = 'N';
        }
        
    pc.printf("\n\rEnter heart rate between 40 and 120 BPM plus RETURN/ENTER: ");
    scanf("%d",&hr);
    pc.printf("Heart rate = %d\n\r", hr);
    if (hr < 40 || hr > 120) {
        hr = 60;
        pc.printf("\n\rError. Heart rate entered was out of range and was reset to 60 BPM.\n\r");
    }
    iso_pts = (int)(21600/hr) - BUFFER_SIZE1; //# of samples between PQRST templates
    iso_pts3 = BUFFER_SIZE1 + 2*iso_pts - BUFFER_SIZE3;
    base_pts = iso_pts;
    pc.getc();  // Clear kbd buffer
    pc.printf("Add noise to ECG - Y or N? ");
    char input1 = pc.putc(pc.getc());
    pc.printf("\n\r");
    switch (input1) {
       case 'Y': case 'y': 
            pc.printf("Add 60 Hz noise? ");
            ampl_60Hz = get_percent_noise() / 100.0f;  
            pc.printf("Add EMG noise? ");
            ampl_rand = get_percent_noise() / 100.0f;
            pc.printf("Add low-frequency baseline noise? ");
            ampl_lf = get_percent_noise() / 100.0f;
            break;
        case 'N': case 'n':
            break;
        default: 
            pc.printf("\n\rEntry error - no noise added\n\r");
    }    
//    pc.printf("Hit ESC key to restart program.\n\r");
}
