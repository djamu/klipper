//
// Generic code for wave driven steppers, pwm / spi / uart / can
// requires low level pmw / spi driver / uart / can drivers
//
// contains LUT lookup and pointer offset code for both freq and amplitude modulation
//
//
//  Copyright (C) 2021  "djamu" Jan Van Haute, adapted for Klipper Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
// 
// drop-in replacement / abstract for driver_a4954.c 

#include "sinewave.c" //

#include "board/irq.h" // irq_disable
#include "board/gpio.h" // gpio_pwm_write
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
//#include "driver_a4954.h" // a4954_oid_lookup
#include "wavedriver.h" // a4954_oid_lookup
#include "sched.h" // DECL_SHUTDOWN
// I'm going to keep these for now...

#define USE_FULL_WAVE 1 // use high torque full wave drive... when needed.
#define USE_TMC 1		// use TMC SPI instead of PWM


// no need to define an ouput struct here yet...
// struct a4954 {
    // struct gpio_out in1, in2, in3, in4;
    // struct gpio_pwm vref12, vref34;
    // uint32_t last_phase;
    // uint8_t flags;
// };
// enum { AF_ENABLED = 1 };

// Generated with the following python script:
//  "".join(["%.0f,"%(math.sin(math.pi*i/512.)*1024-i*4) for i in range(256)])
// static const uint8_t sine_table[256] = {
    // 0,2,5,7,9,11,14,16,18,21,23,25,27,30,32,34,
    // 36,39,41,43,45,48,50,52,54,56,59,61,63,65,67,70,
    // 72,74,76,78,80,82,85,87,89,91,93,95,97,99,101,103,
    // 105,107,109,111,113,115,117,119,121,123,125,127,129,130,132,134,
    // 136,138,139,141,143,145,146,148,150,151,153,155,156,158,160,161,
    // 163,164,166,167,169,170,172,173,174,176,177,179,180,181,182,184,
    // 185,186,187,188,190,191,192,193,194,195,196,197,198,199,200,201,
    // 202,202,203,204,205,206,206,207,208,208,209,210,210,211,211,212,
    // 212,213,213,213,214,214,214,215,215,215,215,215,215,215,216,216,
    // 216,216,215,215,215,215,215,215,214,214,214,214,213,213,212,212,
    // 211,211,210,210,209,208,208,207,206,206,205,204,203,202,201,200,
    // 199,198,197,196,195,193,192,191,190,188,187,186,184,183,181,180,
    // 178,176,175,173,171,170,168,166,164,162,160,158,156,154,152,150,
    // 148,146,143,141,139,137,134,132,129,127,124,122,119,116,114,111,
    // 108,106,103,100,97,94,91,88,85,82,79,76,72,69,66,62,
    // 59,56,52,49,45,42,38,34,31,27,23,20,16,12,8,4,
// }; // why half wave, not quarter ?

// calculte wavetable pointer lookup offset, 
// 

static inline void get_PhPointer()// just inline it, wherever it will end up. calculates the wave pointer offset
// freq modulates drive wave

static uint32_t offsetPointer; // sums additional motor / H-bridge factory offsets from both global and cyclic correction curves 
static int32_t PIIP_Sum; //  sums PIIP result / offsets before clamping it to a maximum range

// uint32_t PhPointer ; // global, 13 bit pointer reference to actual wave
//                         highest 2 bits used for phase lowest 11 lookup as I'm only using a quarter wave
// 
// int32_t PIIP_Out ;    // global, stores result of PIIP controller.
// int32_t PIIP_SpdCmp ; // global, stores velocity related offset. accounts for latency between sensor readout and actual motor propagation.
                         // is velocity related, is similar to DAEC, which unfortunately only accounts for internal latency.
                         // is the actual delay as the sum of SPI transmission of sensor data + internal processing + SPI / PWM
                         // result is a const coeff multiplied by current motor velocity.
                         // at 10kHz sample rate, 3000RPM this ramps up 1 steps ahead, is almost linear

// const int32_t piipRangeMax = 9216 ; // Clamp result to this,  is 2.25 steps, found out by testing, theoretically this should be limtited to 2

// int32_t setPoint ;     // stepped abs motor position, driven by virtual stepper.c or differential SPI subset > see further
// int32_t acc_setPoint ; // global, is calculated accelleration of setpoint, instead klipper could provide this.
// const uint32_t massCoef ; // coeff of axis mass, to compenst
//
// just a thought, since only klippy contains the SPI registry values of SPI devices connected to mcu's, the protocol has means of queueing up SPI commands,
// theoretically this SPI subset could be used to transfer the differential stepper positions at a fixed interval, still as a queue
// the virtual_stepper code could run @ klippy, that transfers it over formentioned SPI subset protocol
// setpoint will still work the same way as an integral, summing the differential values ( but with slightly larger nrs )
// 
// 

#if (USE_PIIPs)
PIIP_Sum = (constrain((PIIP_Out + PIIP_SpdCmp), -(piipRangeMax), piipRangeMax));
PhPointer = (uint32_t)((sensor_data + stepperResolution) * substepsDec) + PIIP_Sum; // keep it simple  offset it 1 revolution to prevent underflow > PIIP_Sum can be negative!
// this is the furthest magnetic field may deviate from setPoint 
PIIP_Sum = PIIP_Out;
#if SPLIT_CORRECTION_CURVE
//offset wavepointer by correction curve(s) for closed loop
// offset @ projected position, not current
//	offsetPointer = (((PhPointer - 2048) / substepsDec) % stepperResolution);
offsetPointer = (((PhPointer - 4096) / substepsDec) % stepperResolution);
//	offsetPointer = (((PhPointer - 6144) / substepsDec) % stepperResolution);
PhPointer -= curve_yValues_lookup[offsetPointer]; // project to desired value
offsetPointer = (((PhPointer - 2048) / substepsDec) % stepperResolution);
PhPointer -= cyclic_avg_offset[(offsetPointer & 127)]; // project to desired value
#else // use combined correction lut
offsetPointer = (((PhPointer - 2048) / substepsDec) % stepperResolution);
PhPointer -= curve_yValues_lookup[offsetPointer]; // project to desired value
#endif
//	PhPointer = PhPointer % stepperPwmRes;

#else
static int32_t inertiaCmp; // inertia offset aka fast pressure advance
// use mass * acc coefficient to predict / compensate motor lag for open loop
inertiaCmp = acc_setPoint * massCoef; // * velocity ?
PIIP_Sum = (constrain((inertiaCmp + PIIP_SpdCmp), -(piipRangeMax), piipRangeMax));
PhPointer = (uint32_t)((setPoint + stepperResolution) * substepsDec) + PIIP_Sum; // keep it simple  offset it 1 revolution to prevent underflow > PIIP_Sum can be negative!
// Currently there's no mapping for open loop
//PhPointer -= curve_yValues_lookup[((inertiaCmp + 0) % stepperResolution)];

#endif

PhPointer &= 0x03FFF; // 4 x 4096, 1 cycle / 4steps is enough info, throw away all other bits

}

// wave / phase lookup & powermanagement
static inline void drive_wave_lookup()
{

static const  uint8_t Phase_table[4]{ B00001010, B00000110, B00000101, B00001001 };// H-bridge polarity sequence
static uint8_t PhaseA = 0;
static uint8_t PhaseA_old = 0;
static uint8_t PhaseB = 0;
static uint8_t PhaseB_old = 0;


#if USE_FULL_WAVE 
	static uint16_t full_wave_A;
	static uint16_t full_wave_B;
#endif

	PhPointerA = PhPointer & 0x0FFF;
	Phase = (PhPointer >> 12) & 3;
	PhaseA = (Phase_table[Phase] & 12);
//	PhaseA = (Phase_table[Phase] >> 2);
	PhaseB = (Phase_table[Phase] & 3);

	if ((bool(PhaseA & 8) == bool(PhaseB & 2)))
	{
		Out_A = wave_array_default[(0x0FFF - PhPointerA)];
		Out_B = wave_array_default[PhPointerA];
		if (!(bool(PhaseA & 8))) {
			Out_A = -Out_A;
			Out_B = -Out_B;
		}
	}
	else
	{
		Out_B = wave_array_default[(0x0FFF - PhPointerA)];
		Out_A = wave_array_default[PhPointerA];
		(bool(PhaseA & 8)) ? Out_B = -Out_B : Out_A = -Out_A;
	}


	if (SWAP_OUTPUTS_A_B)
	{
		Out_Bb = map(Out_A, -2047, 2047, -(tmc_waverange), tmc_waverange);  // 4095 > 248
		Out_Aa = map(Out_B, -2047, 2047, -(tmc_waverange), tmc_waverange);
	}
	else
	{
		Out_Bb = map(Out_B, -2047, 2047, -(tmc_waverange), tmc_waverange);  // 4095 > 248
		Out_Aa = map(Out_A, -2047, 2047, -(tmc_waverange), tmc_waverange);
	}


#if  USE_FULL_WAVE
	//#define fullwave_offset 128
	//#define fullwave_max 192
	//	full_wave = map(tmc_waverange, tmc_min_waverange, tmc_max_burst_waverange, 0, (fullwave_max + fullwave_offset));
	//	full_wave = constrain((full_wave - fullwave_offset), 0, fullwave_max);

#define fullwave_min_pps 176 // 200 = 37500 mm/min.
#define fullwave_max_pps 384 // 256 = 48000 mm/min
#define fullwave_min_pid 3840 // 
#define fullwave_max_pid 4095 // 
#define fullwave_max_A 224 // 208
#define fullwave_max_B 224 // 208

//	if (abs(myPID.spd_Sensor_sum) >= fullwave_min_pps) {
	if ((abs(myPID.spd_Sensor_sum) >= fullwave_min_pps) || (abs(PID_Out) >= fullwave_min_pid)) {
		full_wave_A = map((constrain(abs(myPID.spd_Sensor_sum), fullwave_min_pps, fullwave_max_pps)), fullwave_min_pps, fullwave_max_pps, 0, fullwave_max_A);
		full_wave_B = map((constrain(abs(PID_Out), fullwave_min_pid, fullwave_max_pid)), fullwave_min_pid, fullwave_max_pid, 0, fullwave_max_B);
		if (full_wave_B > full_wave_A) full_wave_A = full_wave_B;
		if (Out_Bb > 0) {
			Out_Bb = ((((256 - full_wave_A) * (abs(Out_Bb))) + (full_wave_A * 255)) >> 8);
		}
		else
		{
			Out_Bb = -((((256 - full_wave_A) * (abs(Out_Bb))) + (full_wave_A * 255)) >> 8);
		}
		if (Out_Aa > 0) {
			Out_Aa = ((((256 - full_wave_A) * (abs(Out_Aa))) + (full_wave_A * 255)) >> 8);
		}
		else
		{
			Out_Aa = -((((256 - full_wave_A) * (abs(Out_Aa))) + (full_wave_A * 255)) >> 8);
		}
	}
#endif
	//if (SWAP_POLARITY_A)
	//{
	//	Out_Aa = -Out_Aa;
	//}

	//(Motor_enable) ? ledcWrite(PWM_CHANNEL_A, Out_Aa) : ledcWrite(PWM_CHANNEL_A, 0); // turn motor off
	//(Motor_enable) ? ledcWrite(PWM_CHANNEL_B, Out_Bb) : ledcWrite(PWM_CHANNEL_B, 0); // turn motor off

	//if (full_wave_8bit)
	//{
	//	//		powerfactor = powerNom;
	//	powerfactor = powerMax;
	//}


	//if ((powerfactor_o >> 4) != (powerfactor >> 4))
	//{
	//	driver.rms_current(powerfactor); 	// double it for direct mode
	//	powerfactor_o = powerfactor;
	//}
	if ((Out_Aa_old != Out_Aa) || (Out_Bb_old != Out_Bb))
	{
		//(Motor_enable) ? driver.XDIRECT(((uint32_t)Out_Aa & COIL_A_bm) | (((uint32_t)Out_Bb << 16) & COIL_B_bm)) : driver.XDIRECT(0); // turn motor off
//		noInterrupts();
		driver.XDIRECT(((uint32_t)Out_Aa & COIL_A_bm) | (((uint32_t)Out_Bb << 16) & COIL_B_bm));
		//		interrupts();
		Out_Aa_old = Out_Aa;
		Out_Bb_old = Out_Bb;
	}
	else
	{
		if (get_tmc_temp) {
			get_tmc_temp = false;
			(tmc_otpw) ? scheduler = !scheduler : scheduler = false;
			if (tmc_otpw) {
				scheduler = !scheduler;
				if (tmc_otw_power_scale > tmc_min_waverange) tmc_otw_power_scale--;
			}
			else {
				scheduler = false;
				if (tmc_otw_power_scale < 256) tmc_otw_power_scale++;
			}
			if (!scheduler) { // 50Hz max
				tmc_otpw = driver.otpw();
			}
			else {
				if ((tmc_otpw) || (tmc_ot)) { // 25Hz max
					tmc_ot = driver.ot();
				}
			}
			// if nothing is done, check for temperature,
		}
	}
	//digitalWrite(Pin_driver[0], bool(PhaseA & 8));
	//digitalWrite(Pin_driver[1], bool(PhaseA & 4));

	//digitalWrite(Pin_driver[2], bool(PhaseB & 2));
	//digitalWrite(Pin_driver[3], bool(PhaseB & 1));

//	PhPointer += stepperPwmRes;

}

#if 
static inline void Xdirect_out(int16_t outA, int16_t outB)
{
	static int16_t Out_Aa_old;
	static int16_t Out_Bb_old;
	//	static uint16_t powerfactor_o;
	static bool scheduler = false;

}

static inline void PWM_out(int16_t outA, int16_t outB)
{
	static int16_t Out_Aa_old;
	static int16_t Out_Bb_old;
	//	static uint16_t powerfactor_o;
	static bool scheduler = false;

}

// Calculate a sine value from a phase (phase is between 0-512)
static uint32_t
lookup_sine(uint32_t phase, uint32_t scale)
{
    uint32_t idx = phase & 0xff;
    if (phase & 0x100)
        idx = ARRAY_SIZE(sine_table) - 1 - idx;
    uint32_t sin = sine_table[idx] + idx * 4;
    return DIV_ROUND_CLOSEST(scale * sin, 1024);
}

// Set the phase and current of the a4954 driver. Caller must disable irqs.
void
a4954_set_phase(struct a4954* a, uint32_t phase, uint32_t scale)
{
    // Determine phase change
    uint32_t last_phase = a->last_phase, phase_xor = last_phase ^ phase;
    a->last_phase = phase;

    // Calculate new coil power
    uint32_t coil1_pow = lookup_sine(phase, scale);
    uint32_t coil2_pow = lookup_sine(phase + 256, scale);

    //output("set_phase lp=%u p=%u px=%u c1=%u c2=%u"
    //       , last_phase, phase, (phase_xor & 0x300)>>8, coil1_pow, coil2_pow);

    // Apply update
    struct gpio_pwm vref12 = a->vref12, vref34 = a->vref34;
    gpio_pwm_write(vref12, coil1_pow);
    gpio_pwm_write(vref34, coil2_pow);
    if (phase_xor & 0x200) {
        struct gpio_out in1 = a->in1, in2 = a->in2;
        gpio_out_toggle_noirq(in1);
        gpio_out_toggle_noirq(in2);
    }
    if ((phase_xor + 256) & 0x200) {
        struct gpio_out in3 = a->in3, in4 = a->in4;
        gpio_out_toggle_noirq(in3);
        gpio_out_toggle_noirq(in4);
    }
}

void
a4954_disable(struct a4954* a)
{
    if (!a->flags)
        // Already disabled
        return;

    struct gpio_pwm vref12 = a->vref12, vref34 = a->vref34;
    gpio_pwm_write(vref12, 0);
    gpio_pwm_write(vref34, 0);
    struct gpio_out in1 = a->in1, in2 = a->in2;
    gpio_out_write(in1, 0);
    gpio_out_write(in2, 0);
    struct gpio_out in3 = a->in3, in4 = a->in4;
    gpio_out_write(in3, 0);
    gpio_out_write(in4, 0);

    a->flags = 0;
}

void
a4954_enable(struct a4954* a)
{
    if (a->flags)
        // Already enabled
        return;
    a->flags = AF_ENABLED;

    uint32_t phase = a->last_phase;
    struct gpio_out in1 = a->in1, in2 = a->in2;
    if (phase & 0x200) {
        gpio_out_write(in1, 1);
        gpio_out_write(in2, 0);
    }
    else {
        gpio_out_write(in1, 0);
        gpio_out_write(in2, 1);
    }
    struct gpio_out in3 = a->in3, in4 = a->in4;
    if ((phase + 256) & 0x200) {
        gpio_out_write(in3, 1);
        gpio_out_write(in4, 0);
    }
    else {
        gpio_out_write(in3, 0);
        gpio_out_write(in4, 1);
    }
}

void
command_config_a4954(uint32_t* args)
{
    struct gpio_out in1 = gpio_out_setup(args[1], 0);
    struct gpio_out in2 = gpio_out_setup(args[2], 0);
    struct gpio_out in3 = gpio_out_setup(args[3], 0);
    struct gpio_out in4 = gpio_out_setup(args[4], 0);
    struct gpio_pwm vref12 = gpio_pwm_setup(args[5], 1, 0);
    struct gpio_pwm vref34 = gpio_pwm_setup(args[6], 1, 0);
    struct a4954* a = oid_alloc(args[0], command_config_a4954, sizeof(*a));
    a->in1 = in1;
    a->in2 = in2;
    a->in3 = in3;
    a->in4 = in4;
    a->vref12 = vref12;
    a->vref34 = vref34;
}
DECL_COMMAND(command_config_a4954,
    "config_a4954 oid=%c in1_pin=%u in2_pin=%u in3_pin=%u in4_pin=%u"
    " vref12_pin=%u vref34_pin=%u");

struct a4954*
    a4954_oid_lookup(uint8_t oid)
{
    return oid_lookup(oid, command_config_a4954);
}

void
a4954_shutdown(void)
{
    uint8_t i;
    struct a4954* a;
    foreach_oid(i, a, command_config_a4954) {
        a4954_disable(a);
    }
}
DECL_SHUTDOWN(a4954_shutdown);