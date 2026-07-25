/*
 * diag_btn.c - find which GPIO each button is actually wired to.
 *
 * For boards where KEY1/KEY2 are not on the hardcoded PA1/PB22. Standalone:
 * no USB, no BLE, no UART. Output is the LED matrix itself, using the
 * empirically confirmed B1144C anode table.
 *
 * How it works
 * ------------
 * Every GPIO that is NOT part of the LED matrix is a button candidate. The
 * scanner watches all of them and, when one reads as pressed, lights the
 * column pair whose index equals that candidate's index. Count the lit
 * columns and look the number up in the table below.
 *
 * Buttons can be either polarity - stock btn_init() uses pull-DOWN on KEY1
 * (active high) and pull-UP on KEY2 (active low) - so the scan alternates:
 *
 *   Phase PU (8 s): candidates pulled up,   a LOW  reading = pressed
 *   Phase PD (8 s): candidates pulled down, a HIGH reading = pressed
 *                   ...and columns 43,44 light as a phase marker.
 *
 * So: hold a button, watch for ~16 s, note which columns light and whether
 * the 43,44 marker was lit at the same time.
 *
 * Candidate index -> columns that light:
 *   0 PA0 ->1,2    1 PA1 ->3,4    2 PA2 ->5,6    3 PA3 ->7,8
 *   4 PA5 ->9,10   5 PA6 ->11,12  6 PA7 ->13,14  7 PA13->15,16
 *   8 PA14->17,18  9 PB16->19,20 10 PB17->21,22 11 PB22->23,24
 *  12 PB23->25,26
 *
 * PA1 is the documented KEY1, PB22 the documented KEY2, PA13 the KEY3/KEY4
 * ADC divider. They are included so a working stock wiring is visible too.
 *
 * Never touched: the 23 LED pins, PA8/PA9 (debug UART), PB10/PB11 (USB D-/D+).
 * Candidates are only ever inputs - nothing here drives them.
 *
 * Build (from repo root):
 *   make TARGET=badgemagic-diagbtn BUILD_DIR=build-diagbtn \
 *     C_SOURCES="CH5xx_ble_firmware_library/StdPeriphDriver/CH58x_sys.c \
 *                CH5xx_ble_firmware_library/StdPeriphDriver/CH58x_clk.c \
 *                CH5xx_ble_firmware_library/StdPeriphDriver/CH58x_gpio.c \
 *                CH5xx_ble_firmware_library/RVMSIS/core_riscv.c \
 *                src/diag_btn.c" \
 *     PREFIX=<mounriver>/RISC-V_Embedded_GCC/bin/riscv-none-embed-
 */

#include "CH58x_common.h"

#define BANK_A 0
#define BANK_B 1

typedef struct {
	uint8_t  bank;
	uint32_t mask;
} dpin_t;

/*
 * Confirmed B1144C LED pin table (see leddrv.c). Entries 0..21 are the
 * anodes for column pairs (2k+1, 2k+2); entry 22 is cathode-only. All 23
 * are used as cathodes when strobing.
 */
static const dpin_t led[] = {
	{BANK_A, GPIO_Pin_15}, /*  0 -> cols 1,2   */
	{BANK_B, GPIO_Pin_18}, /*  1 -> cols 3,4   */
	{BANK_B, GPIO_Pin_0},  /*  2 -> cols 5,6   */
	{BANK_B, GPIO_Pin_7},  /*  3 -> cols 7,8   */
	{BANK_A, GPIO_Pin_12}, /*  4 -> cols 9,10  */
	{BANK_A, GPIO_Pin_10}, /*  5 -> cols 11,12 */
	{BANK_A, GPIO_Pin_11}, /*  6 -> cols 13,14 */
	{BANK_B, GPIO_Pin_9},  /*  7 -> cols 15,16 */
	{BANK_B, GPIO_Pin_8},  /*  8 -> cols 17,18 */
	{BANK_B, GPIO_Pin_15}, /*  9 -> cols 19,20 */
	{BANK_B, GPIO_Pin_14}, /* 10 -> cols 21,22 */
	{BANK_B, GPIO_Pin_13}, /* 11 -> cols 23,24 */
	{BANK_B, GPIO_Pin_12}, /* 12 -> cols 25,26 */
	{BANK_B, GPIO_Pin_5},  /* 13 -> cols 27,28 */
	{BANK_A, GPIO_Pin_4},  /* 14 -> cols 29,30 */
	{BANK_B, GPIO_Pin_3},  /* 15 -> cols 31,32 */
	{BANK_B, GPIO_Pin_4},  /* 16 -> cols 33,34 */
	{BANK_B, GPIO_Pin_2},  /* 17 -> cols 35,36 */
	{BANK_B, GPIO_Pin_1},  /* 18 -> cols 37,38 */
	{BANK_B, GPIO_Pin_6},  /* 19 -> cols 39,40 */
	{BANK_B, GPIO_Pin_21}, /* 20 -> cols 41,42 */
	{BANK_B, GPIO_Pin_20}, /* 21 -> cols 43,44 */
	{BANK_B, GPIO_Pin_19}, /* 22 -> cathode only */
};

#define NLED (sizeof(led) / sizeof(led[0]))

#define MARKER_PAIR (21) /* cols 43,44 - lit during the pull-down phase */

/* Every non-LED, non-USB, non-UART GPIO. */
static const dpin_t cand[] = {
	{BANK_A, GPIO_Pin_0},  /*  0 */
	{BANK_A, GPIO_Pin_1},  /*  1  documented KEY1 */
	{BANK_A, GPIO_Pin_2},  /*  2 */
	{BANK_A, GPIO_Pin_3},  /*  3 */
	{BANK_A, GPIO_Pin_5},  /*  4 */
	{BANK_A, GPIO_Pin_6},  /*  5 */
	{BANK_A, GPIO_Pin_7},  /*  6 */
	{BANK_A, GPIO_Pin_13}, /*  7  KEY3/KEY4 ADC divider */
	{BANK_A, GPIO_Pin_14}, /*  8 */
	{BANK_B, GPIO_Pin_16}, /*  9 */
	{BANK_B, GPIO_Pin_17}, /* 10 */
	{BANK_B, GPIO_Pin_22}, /* 11  documented KEY2 */
	{BANK_B, GPIO_Pin_23}, /* 12 */
};

#define NCAND (sizeof(cand) / sizeof(cand[0]))

#define SLOT_US   (200)
#define PHASE_MS  (8000)

static void led_float(const dpin_t *p)
{
	if (p->bank == BANK_A)
		GPIOA_ModeCfg(p->mask, GPIO_ModeIN_Floating);
	else
		GPIOB_ModeCfg(p->mask, GPIO_ModeIN_Floating);
}

static void led_high(const dpin_t *p)
{
	if (p->bank == BANK_A) {
		GPIOA_SetBits(p->mask);
		GPIOA_ModeCfg(p->mask, GPIO_ModeOut_PP_20mA);
	} else {
		GPIOB_SetBits(p->mask);
		GPIOB_ModeCfg(p->mask, GPIO_ModeOut_PP_20mA);
	}
}

static void led_low(const dpin_t *p)
{
	if (p->bank == BANK_A) {
		GPIOA_ResetBits(p->mask);
		GPIOA_ModeCfg(p->mask, GPIO_ModeOut_PP_5mA);
	} else {
		GPIOB_ResetBits(p->mask);
		GPIOB_ModeCfg(p->mask, GPIO_ModeOut_PP_5mA);
	}
}

static void leds_all_float(void)
{
	unsigned k;

	for (k = 0; k < NLED; k++)
		led_float(&led[k]);
}

/* One strobe pass over column pair d. Exactly one LED lit at a time. */
static void light_pair(unsigned d)
{
	unsigned j;

	leds_all_float();
	led_high(&led[d]);

	for (j = 0; j < NLED; j++) {
		if (j == d)
			continue;
		led_low(&led[j]);
		DelayUs(SLOT_US);
		led_float(&led[j]);
	}

	leds_all_float();
}

/* Configure every candidate as an input, pulled to the given side. */
static void cand_config(int pull_up)
{
	GPIOModeTypeDef m = pull_up ? GPIO_ModeIN_PU : GPIO_ModeIN_PD;
	unsigned k;

	for (k = 0; k < NCAND; k++) {
		if (cand[k].bank == BANK_A)
			GPIOA_ModeCfg(cand[k].mask, m);
		else
			GPIOB_ModeCfg(cand[k].mask, m);
	}
}

/* Bitmap of candidates currently reading as pressed for this polarity. */
static uint32_t cand_scan(int pull_up)
{
	uint32_t hits = 0;
	unsigned k;

	for (k = 0; k < NCAND; k++) {
		uint32_t v = (cand[k].bank == BANK_A)
			   ? GPIOA_ReadPortPin(cand[k].mask)
			   : GPIOB_ReadPortPin(cand[k].mask);

		/* pulled up  -> pressed reads LOW
		 * pulled down-> pressed reads HIGH */
		if (pull_up ? (v == 0) : (v != 0))
			hits |= (1u << k);
	}

	return hits;
}

static void run_phase(int pull_up, uint32_t ms)
{
	uint32_t frame_us = (uint32_t)SLOT_US * (NLED - 1);
	uint32_t frames = (ms * 1000u) / frame_us;

	cand_config(pull_up);

	while (frames--) {
		uint32_t hits = cand_scan(pull_up);
		unsigned k;
		int drew = 0;

		for (k = 0; k < NCAND; k++) {
			if (hits & (1u << k)) {
				light_pair(k);
				drew = 1;
			}
		}

		if (!pull_up) {
			light_pair(MARKER_PAIR);
			drew = 1;
		}

		if (!drew)
			DelayUs(frame_us);
	}

	leds_all_float();
}

int main(void)
{
	SetSysClock(CLK_SOURCE_PLL_60MHz);
	leds_all_float();

	while (1) {
		run_phase(1, PHASE_MS); /* pull-up:   active-low buttons  */
		run_phase(0, PHASE_MS); /* pull-down: active-high buttons */
	}
}
