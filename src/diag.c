/*
 * diag.c - empirical Charlieplex pin mapping.
 *
 * For badges whose pinout is not one of the led_pins[] tables in leddrv.c.
 * Standalone: no USB, no BLE, no UART, no buttons - none of which can be
 * trusted on an unknown board revision. The LED sweep itself is the output.
 *
 * What it does, forever:
 *
 *   1. MARKER: three full-display flashes  -> start of a cycle
 *   2. 26 steps, 3 s each, 250 ms dark between: step i drives candidate pin i
 *      as the anode and multiplexes every other candidate as cathode, so the
 *      whole LED group belonging to pin i lights up.
 *
 * Film one cycle and note which columns light on each step. Step order is the
 * fixed order of cand[] below. That yields the real anode->column mapping,
 * which is what led_pins[] encodes.
 *
 * SAFETY: exactly one LED is powered at any instant - cathodes are strobed one
 * at a time rather than held. Peak current through a driven pin is one LED's
 * worth, well under the production driver, which holds up to 22 on at once.
 *
 * Deliberately NOT driven, because driving them breaks things or bricks the
 * session:
 *   PB10, PB11 - USB D-/D+
 *   PB22       - KEY2, and DOWNLOAD_CFG bootloader-entry pin (see CH582.md)
 *   PA1        - KEY1
 *   PA8, PA9   - debug UART
 *   PA13       - KEY3/KEY4 ADC divider
 *
 * Build (from repo root):
 *   make TARGET=badgemagic-diag BUILD_DIR=build-diag \
 *     C_SOURCES="CH5xx_ble_firmware_library/StdPeriphDriver/CH58x_sys.c \
 *                CH5xx_ble_firmware_library/StdPeriphDriver/CH58x_clk.c \
 *                CH5xx_ble_firmware_library/StdPeriphDriver/CH58x_gpio.c \
 *                CH5xx_ble_firmware_library/RVMSIS/core_riscv.c \
 *                src/diag.c" \
 *     PREFIX=<mounriver>/RISC-V_Embedded_GCC/bin/riscv-none-embed-
 */

#include "CH58x_common.h"

#define BANK_A 0
#define BANK_B 1

typedef struct {
	uint8_t     bank;
	uint32_t    mask;
	const char *name; /* documents the sweep order; unused at runtime */
} dpin_t;

/*
 * Union of every pin appearing in any led_pins[] variant, in a fixed order:
 * port A ascending, then port B ascending. Step numbers below refer to this
 * order, 1-based.
 */
static const dpin_t cand[] = {
	{BANK_A, GPIO_Pin_4,  "PA4"},   /*  1 */
	{BANK_A, GPIO_Pin_10, "PA10"},  /*  2 */
	{BANK_A, GPIO_Pin_11, "PA11"},  /*  3 */
	{BANK_A, GPIO_Pin_12, "PA12"},  /*  4 */
	{BANK_A, GPIO_Pin_15, "PA15"},  /*  5 */
	{BANK_B, GPIO_Pin_0,  "PB0"},   /*  6 */
	{BANK_B, GPIO_Pin_1,  "PB1"},   /*  7 */
	{BANK_B, GPIO_Pin_2,  "PB2"},   /*  8 */
	{BANK_B, GPIO_Pin_3,  "PB3"},   /*  9 */
	{BANK_B, GPIO_Pin_4,  "PB4"},   /* 10 */
	{BANK_B, GPIO_Pin_5,  "PB5"},   /* 11 */
	{BANK_B, GPIO_Pin_6,  "PB6"},   /* 12 */
	{BANK_B, GPIO_Pin_7,  "PB7"},   /* 13 */
	{BANK_B, GPIO_Pin_8,  "PB8"},   /* 14 */
	{BANK_B, GPIO_Pin_9,  "PB9"},   /* 15 */
	{BANK_B, GPIO_Pin_12, "PB12"},  /* 16 */
	{BANK_B, GPIO_Pin_13, "PB13"},  /* 17 */
	{BANK_B, GPIO_Pin_14, "PB14"},  /* 18 */
	{BANK_B, GPIO_Pin_15, "PB15"},  /* 19 */
	{BANK_B, GPIO_Pin_16, "PB16"},  /* 20 */
	{BANK_B, GPIO_Pin_17, "PB17"},  /* 21 */
	{BANK_B, GPIO_Pin_18, "PB18"},  /* 22 */
	{BANK_B, GPIO_Pin_19, "PB19"},  /* 23 */
	{BANK_B, GPIO_Pin_20, "PB20"},  /* 24 */
	{BANK_B, GPIO_Pin_21, "PB21"},  /* 25 */
	{BANK_B, GPIO_Pin_23, "PB23"},  /* 26 */
};

#define NCAND (sizeof(cand) / sizeof(cand[0]))

/* Strobe slots. Step: 25 cathodes * 200us = 5ms/frame = 200Hz, flicker-free.
 * Marker: 26 * 25 * 40us = 26ms/frame = ~38Hz, good enough for a flash. */
#define STEP_SLOT_US (200)
#define MARK_SLOT_US (40)

#define STEP_MS      (3000)
#define GAP_MS       (250)
#define MARK_ON_MS   (800)

static void pin_float(const dpin_t *p)
{
	if (p->bank == BANK_A)
		GPIOA_ModeCfg(p->mask, GPIO_ModeIN_Floating);
	else
		GPIOB_ModeCfg(p->mask, GPIO_ModeIN_Floating);
}

static void pin_high(const dpin_t *p)
{
	if (p->bank == BANK_A) {
		GPIOA_SetBits(p->mask);
		GPIOA_ModeCfg(p->mask, GPIO_ModeOut_PP_20mA);
	} else {
		GPIOB_SetBits(p->mask);
		GPIOB_ModeCfg(p->mask, GPIO_ModeOut_PP_20mA);
	}
}

static void pin_low(const dpin_t *p)
{
	if (p->bank == BANK_A) {
		GPIOA_ResetBits(p->mask);
		GPIOA_ModeCfg(p->mask, GPIO_ModeOut_PP_5mA);
	} else {
		GPIOB_ResetBits(p->mask);
		GPIOB_ModeCfg(p->mask, GPIO_ModeOut_PP_5mA);
	}
}

static void all_float(void)
{
	unsigned k;

	for (k = 0; k < NCAND; k++)
		pin_float(&cand[k]);
}

/* Strobe every cathode against anode i once. One LED lit at a time. */
static void frame_anode(unsigned i, uint16_t slot_us)
{
	unsigned j;

	for (j = 0; j < NCAND; j++) {
		if (j == i)
			continue;
		pin_low(&cand[j]);
		DelayUs(slot_us);
		pin_float(&cand[j]);
	}
}

/* Light everything belonging to anode i for ms milliseconds. */
static void sweep_anode(unsigned i, uint32_t ms)
{
	uint32_t frame_us = (uint32_t)STEP_SLOT_US * (NCAND - 1);
	uint32_t frames = (ms * 1000u) / frame_us;

	all_float();
	pin_high(&cand[i]);

	while (frames--)
		frame_anode(i, STEP_SLOT_US);

	all_float();
}

/* Whole display on, by cycling every anode. Used as the cycle marker. */
static void all_on(uint32_t ms)
{
	uint32_t frame_us = (uint32_t)MARK_SLOT_US * NCAND * (NCAND - 1);
	uint32_t frames = (ms * 1000u) / frame_us;
	unsigned i;

	if (!frames)
		frames = 1;

	while (frames--) {
		for (i = 0; i < NCAND; i++) {
			all_float();
			pin_high(&cand[i]);
			frame_anode(i, MARK_SLOT_US);
		}
	}

	all_float();
}

static void dark(uint32_t ms)
{
	all_float();
	while (ms > 1000) {
		DelayMs(1000);
		ms -= 1000;
	}
	DelayMs((uint16_t)ms);
}

int main(void)
{
	unsigned i, f;

	SetSysClock(CLK_SOURCE_PLL_60MHz);
	all_float();

	while (1) {
		/* Marker: three full-display flashes = a cycle is starting. */
		for (f = 0; f < 3; f++) {
			all_on(MARK_ON_MS);
			dark(GAP_MS);
		}
		dark(1000);

		/* Steps 1..NCAND, in cand[] order. */
		for (i = 0; i < NCAND; i++) {
			sweep_anode(i, STEP_MS);
			dark(GAP_MS);
		}
	}
}
