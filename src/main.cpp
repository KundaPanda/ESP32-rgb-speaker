#include "soc/timer_group_reg.h"
#include "soc/timer_group_struct.h"
#include <Arduino.h>
#include <FastLED.h>
#include <LEDMatrix.h>
#include <NeoPixelBus.h>
#include <WiFi.h>
#include <analogWrite.h>
#include <arduinoFFT.h>
#include <esp32-hal-cpu.h>

#define tempPin 27
#define fanPin 12
#define micPin 32
#define NSAMPLES 512
#define SLEEP_TIME_MS 1000
#define STATE_SWITCH_CYCLES 7

#define DATA_PIN 25
#define COLOR_ORDER RGB
#define CHIPSET WS2812B
#define MATRIX_WIDTH 16
#define MATRIX_HEIGHT 16
#define MATRIX_TYPE VERTICAL_ZIGZAG_MATRIX
const int MATRIX_SIZE = (MATRIX_WIDTH * MATRIX_HEIGHT);
const int NUMPIXELS = MATRIX_SIZE;
cLEDMatrix<MATRIX_WIDTH, MATRIX_HEIGHT, MATRIX_TYPE> leds;
DEFINE_GRADIENT_PALETTE(rainbow_gp){
	0, 126, 1, 142,
	25, 171, 1, 26,
	48, 224, 9, 1,
	71, 237, 138, 1,
	94, 52, 173, 1,
	117, 1, 201, 1,
	140, 1, 211, 54,
	163, 1, 124, 168,
	186, 1, 8, 149,
	209, 12, 1, 151,
	232, 12, 1, 151,
	255, 171, 1, 190
};

DEFINE_GRADIENT_PALETTE(spellbound_gp){
	0, 232, 235, 40,
	12, 157, 248, 46,
	25, 100, 246, 51,
	45, 53, 250, 33,
	63, 18, 237, 53,
	81, 11, 211, 162,
	94, 18, 147, 214,
	101, 43, 124, 237,
	112, 49, 75, 247,
	127, 49, 75, 247,
	140, 92, 107, 247,
	150, 120, 127, 250,
	163, 130, 138, 252,
	173, 144, 131, 252,
	186, 148, 112, 252,
	196, 144, 37, 176,
	211, 113, 18, 87,
	221, 163, 33, 53,
	234, 255, 101, 78,
	247, 229, 235, 46,
	255, 229, 235, 46
};

CRGBPalette256 currentPalette = rainbow_gp;
int paletteProgress = 0;
int paletteStepSpectrum = 4;
int paletteStep = 1;
const int PALETTE_STEP_COUNTDOWN = 4;
int paletteStepCountdown = PALETTE_STEP_COUNTDOWN;
const CRGB off(0, 0, 0);
const CRGB white(255, 255, 255);
const CRGB red(255, 0, 0);

const float MATRIX_FPS = 60.0;
const float CALIBRATE_FREQ = 12000.0;
const float IDLE_MIC_VOLTAGE = 1239.0;
const int IDLE_READING = 1267;
const int LOW_VOLTAGE_ACTUAL = 8;
const int LOW_VOLTAGE_READING = 220;
const float HIGH_VOLTAGE = 3227.0;
const double SAMPLE_RATE = 38000.0;
const double SAMPLE_TIME = (1.0 * 1000 * 1000 / SAMPLE_RATE);
const bool DEBUG = false;

const float BASE_THRESHOLD = 5500.0;
const float MID_THRESHOLD = 15000.0;
const float HIGH_THRESHOLD = 10000.0;
float baseThreshold = BASE_THRESHOLD;
float midThreshold = MID_THRESHOLD;
float highThreshold = HIGH_THRESHOLD;
TaskHandle_t MatrixTask;
typedef ColumnMajorAlternatingLayout MyPanelLayout;
typedef ColumnMajorLayout MyTilesLayout;
const double THRESHOLD = 500.0;
const double FREQ_THRESHOLD = 40.0;
const double SPECTRUM_THRESHOLD = 20000;

// make sure to set these panel values to the sizes of yours
// const uint8_t PanelWidth = 16; // 8 pixel x 8 pixel matrix of leds
// const uint8_t PanelHeight = 16;
// const uint16_t PixelCount = PanelWidth * PanelHeight;
// const uint8_t PixelPin = 25;
// NeoTiles<MyPanelLayout, MyTilesLayout> tiles(
//     PanelWidth,
//     PanelHeight,
//     1,
//     1);
// NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(PixelCount, PixelPin);
// RgbColor red(15, 0, 0);
// RgbColor green(0, 15, 0);
// RgbColor blue(0, 0, 15);
// RgbColor white(255, 255, 255);
// HsbColor whiteHsb(0.0, 0.0, 1.0);
// RgbColor off(0, 0, 0);
// HsbColor rainbowCurrent(0.0, 1.0, 1.0);
// const float rainbowStep = 0.001;
// const float rainbowStepSpectrum = 0.01;

arduinoFFT FFT;
float peakBand;
float peakValue;
float maximum;
double real[NSAMPLES];
double imag[NSAMPLES];

typedef enum {
	STAR,
	STRIPE
} animationType;
typedef struct {
	short state;
	short stateSwitchCycles;
	animationType type;
	bool available;
} animation;
void (*chosenAnimation)(void *);
bool addAnimations = false;
animation animationsArray[16][16];

/**
 * bass - 20-250
 * mids - 250-2500
 * upper mids - 2500-5000
 * highs - 5000-20000
 **/
const int NBANDS = 16;
const int BANDS[NBANDS] = { 200, 450, 700, 900, 1200, 1500, 1800, 2300, 2800, 3500, 4700, 5500, 6700, 8000, 12000, 20000 };
const int lastBaseIndex = 1;
const int lastMidIndex = 7;
const int lastHighIndex = 15;
const int functionSwitchTimeMax = 60000;
const int functionSwitchTimeMin = 15000;
int functionSwitchTime = rand() % (functionSwitchTimeMax - functionSwitchTimeMin) + functionSwitchTimeMin;
int lastSwitchTime = 0;
double bandValues[NBANDS];
unsigned long last = 0;
int animations = 0;
const int maxAnimations = 40;
int baseCycles = 0;
int baseCyclesThreshold = 60;

void getPeak(void) {
	peakValue = 0;
	peakBand = 0;
	for (int i = 0; i < NBANDS; i++) {
		if (peakValue < bandValues[i]) {
			peakValue = bandValues[i];
			peakBand = BANDS[i];
		}
	}
}

void doFFT(void) {
	FFT = arduinoFFT(real, imag, NSAMPLES, SAMPLE_RATE);
	FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
	FFT.Compute(FFT_FORWARD);
	FFT.ComplexToMagnitude();
	getPeak();
	maximum = (peakValue > SPECTRUM_THRESHOLD) ? peakValue : SPECTRUM_THRESHOLD;
}

double analogToVoltage(uint16_t analog) {
	return (HIGH_VOLTAGE * analog) / 4096.0;
}

int voltageToAnalog(double voltage) {
	return (4096.0 * voltage) / HIGH_VOLTAGE;
}

double calibrate_silence(void) {
	int samples = 8192;
	double result = 0;
	for (int i = 0; i < samples; i++) {
		result += 1.0 * analogRead(micPin);
		;
	}
	result /= samples;
	if (DEBUG) {
		Serial.printf("Silence result: %lf\n", result);
	}
	return result;
}

void getSamples(int amount) {
	float freeTime = 0;
	for (int i = 0; i < amount; i++) {
		last = micros();
		real[i] = 1.0 * (analogRead(micPin) - IDLE_READING);
		while (SAMPLE_TIME > micros() - last) {
			freeTime += 1.0;
		}
	}
	if (DEBUG) {
		Serial.printf("Increments: %f = %f per one read.\n", freeTime, freeTime / amount);
	}
}

double getSampleSpeed() {
	int samples = 10000;
	double start = micros();
	for (int i = 0; i < samples; i++) {
		(analogRead(micPin) - IDLE_READING) * 1.0;
	}
	double time = micros() - start;
	double speed = samples / time;
	time = (double)samples / time;
	if (DEBUG) {
		Serial.printf("Time for one sample: %lfus, sample speed: %lf/s\n", time, speed * 1000 * 1000);
	}
	return speed;
}

void showSpectrum(void *params) {
	if (DEBUG) {
		Serial.printf("Showing spectrum\n");
	}
	// float start = millis();
	double values[NBANDS];
	CRGB colors[NBANDS];
	// HsbColor current = rainbowCurrent;
	for (int i = 0; i < NBANDS; i++) {
		values[i] = (maximum * (16 - i) / 15);
		colors[i] = ColorFromPalette(currentPalette, paletteProgress + i * paletteStepSpectrum);
	}
	fill_solid(FastLED.leds(), MATRIX_SIZE, off);
	// strip.ClearTo(off);
	for (int x = 0; x < NBANDS; x++) {
		for (int y = 15; y >= 0; y--) {
			if (bandValues[x] >= values[y]) {
				// strip.SetPixelColor(tiles.Map(x, y), colors[15 - y]);
				leds.DrawPixel(x, y, colors[15 - y]);
			}
		}
		// strip.SetPixelColor(15 * 15, blue);
	}
	FastLED.show();
	// strip.Show();
}

void showAnimations(void) {
	CRGB inverse = ColorFromPalette(currentPalette, 255 - paletteProgress + 30);
	// HsbColor inverseColor(fmodf(rainbowCurrent.H + 0.3, 1.0), 0.0, 0.0);
	// HsbColor color(0.0, 0.0, 0.0);
	if (baseCycles > 0) {
		baseCycles--;
		fill_solid(FastLED.leds(), MATRIX_SIZE, ColorFromPalette(currentPalette, paletteProgress));
	} else {
		fill_solid(FastLED.leds(), MATRIX_SIZE, off);
	}
	// strip.ClearTo(color);
	for (int x = 0; x < MATRIX_WIDTH; x++) {
		for (int y = 0; y < MATRIX_HEIGHT; y++) {
			animation *anim = &animationsArray[y][x];
			if (anim->available)
				continue;
			switch (anim->type) {
			case STAR: {
				switch (anim->state) {
				case 0:
				case 2: {
					// strip.SetPixelColor(tiles.Map(x, y), white);
					leds.DrawPixel(x, y, white);
					anim->stateSwitchCycles--;
					if (anim->stateSwitchCycles == 0) {
						anim->state++;
						anim->stateSwitchCycles = STATE_SWITCH_CYCLES;
					}
					break;
				}
				case 1: {
					leds.DrawPixel(x + 1, y, white);
					leds.DrawPixel(x - 1, y, white);
					leds.DrawPixel(x, y + 1, white);
					leds.DrawPixel(x, y - 1, white);
					// strip.SetPixelColor(tiles.Map(x + 1, y), white);
					// strip.SetPixelColor(tiles.Map(x - 1, y), white);
					// strip.SetPixelColor(tiles.Map(x, y + 1), white);
					// strip.SetPixelColor(tiles.Map(x, y - 1), white);
					anim->stateSwitchCycles--;
					if (anim->stateSwitchCycles == 0) {
						anim->state++;
						anim->stateSwitchCycles = STATE_SWITCH_CYCLES;
					}
					break;
				}
				case 3: {
					anim->available = true;
					animations--;
					break;
				}
				}
				break;
			}
			case STRIPE: {
				switch (anim->state) {
				case 0: {
					leds.DrawPixel(x, y, inverse);
					// strip.SetPixelColor(tiles.Map(x, y), inverseColor);
					anim->stateSwitchCycles--;
					if (anim->stateSwitchCycles == 0) {
						anim->state++;
						anim->stateSwitchCycles = STATE_SWITCH_CYCLES;
					}
					break;
				}
				case 1: {
					leds.DrawPixel(x, y, inverse);
					leds.DrawPixel(x, y + 1, inverse);
					leds.DrawPixel(x, y - 1, inverse);
					// strip.SetPixelColor(tiles.Map(x, y), inverseColor);
					// strip.SetPixelColor(tiles.Map(x, y + 1), inverseColor);
					// strip.SetPixelColor(tiles.Map(x, y - 1), inverseColor);
					anim->stateSwitchCycles--;
					if (anim->stateSwitchCycles == 0) {
						anim->state++;
						anim->stateSwitchCycles = STATE_SWITCH_CYCLES;
					}
					break;
				}
				case 2: {
					leds.DrawPixel(x, y + 1, inverse);
					leds.DrawPixel(x, y - 1, inverse);
					// strip.SetPixelColor(tiles.Map(x, y + 1), inverseColor);
					// strip.SetPixelColor(tiles.Map(x, y - 1), inverseColor);
					anim->stateSwitchCycles--;
					if (anim->stateSwitchCycles == 0) {
						anim->state++;
						anim->stateSwitchCycles = STATE_SWITCH_CYCLES;
					}
					break;
				}
				case 3: {
					anim->available = true;
					animations--;
					break;
				}
				}
				break;
			}
			}
		}
	}
}

void addAnimation(animationType type) {
	if (animations >= maxAnimations) {
		return;
	}
	int pixel = rand() % (MATRIX_SIZE);
	while (animationsArray[pixel % MATRIX_WIDTH][pixel / MATRIX_WIDTH].available == false) {
		//TODO better algorithm for placement
		pixel = (pixel + 1) % (MATRIX_SIZE);
	}
	int x = pixel % MATRIX_WIDTH;
	int y = pixel / MATRIX_WIDTH;
	animationsArray[y][x].available = false;
	animationsArray[y][x].state = 0;
	animationsArray[y][x].type = type;
	animationsArray[y][x].stateSwitchCycles = STATE_SWITCH_CYCLES;
	animations++;
}

void pulseToBase(void *params) {
	for (int i = 0; i < NBANDS; i++) {
		if (i < lastBaseIndex) {
			if (bandValues[i] > baseThreshold) {
				if (!(baseCycles > baseCyclesThreshold)) {
					baseCycles = 15;
				}
			}
		} else if (i < lastMidIndex) {
			if (bandValues[i] > midThreshold) {
				if (addAnimations)
					addAnimation(STAR);
			}
		} else {
			if (bandValues[i] > highThreshold) {
				if (addAnimations)
					addAnimation(STRIPE);
			}
		}
	}
	showAnimations();
	FastLED.show();
	// strip.Show();
	addAnimations = false;
}

void (*animationTypes[])(void *) = { showSpectrum, pulseToBase };

void callMatrixFunction(void *params) {
	unsigned long matrixStart;
	float voltage;
	for (;;) {
		TIMERG1.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
		TIMERG1.wdt_feed = 1;
		TIMERG1.wdt_wprotect = 0;
		matrixStart = millis();
		if (abs(millis() - lastSwitchTime) > functionSwitchTime) {
			lastSwitchTime = millis();
			std::random_shuffle(animationTypes, std::end(animationTypes));
			chosenAnimation = animationTypes[0];
			animations = 0;
			if (DEBUG) {
				voltage = 0;
				for (int i = 0; i < 64; i++) {
					voltage += (HIGH_VOLTAGE * (float)analogRead(tempPin)) / 4096.0;
				}
				voltage /= 64;
				Serial.printf("Temp sensor voltage: %f mV\n", voltage);
				//TODO: temperature measurement + fan curve + fan output (waiting for parts to arrive) -> remove DEBUG
			}
		}
		paletteStepCountdown--;
		if (paletteStepCountdown == 0) {
			paletteProgress = (paletteProgress + paletteStep) % 256;
			paletteStepCountdown = PALETTE_STEP_COUNTDOWN;
		}
		//TODO: debug pulsetobase, remove!!!!
		// chosenAnimation = pulseToBase;
		chosenAnimation(params);
		functionSwitchTime = rand() % (functionSwitchTimeMax - functionSwitchTimeMin) + functionSwitchTimeMin;
		// rainbowCurrent.H = fmodf((rainbowCurrent.H + rainbowStep), 1.0);
		vTaskDelay(1 / portTICK_PERIOD_MS);
		while (abs(millis() - matrixStart) < 1000 / MATRIX_FPS)
			;
	}
}

void setup() {
	setCpuFrequencyMhz(240);
	Serial.begin(115200);
	WiFi.mode(WIFI_OFF);
	btStop();
	for (int i = 0; i < MATRIX_HEIGHT; i++) {
		for (int j = 0; j < MATRIX_WIDTH; j++) {
			animationsArray[i][j].available = true;
		}
	}
	pinMode(micPin, INPUT);
	pinMode(tempPin, INPUT);
	pinMode(fanPin, OUTPUT);
	analogReadResolution(12);
	analogWriteResolution(fanPin, 10);
	analogWriteFrequency(fanPin, 40000);
	// calibrate_silence();
	FastLED.addLeds<CHIPSET, DATA_PIN, COLOR_ORDER>(leds[0], leds.Size()).setCorrection(TypicalSMD5050);
	FastLED.setBrightness(127);
	FastLED.clear(true);
	for (int i = 1; i < sqrt(pow(MATRIX_WIDTH, 2) + pow(MATRIX_HEIGHT, 2)); i++) {
		leds.DrawFilledCircle(0, 0, i, red);
		FastLED.show();
		delay(60);
	}
	fill_solid(FastLED.leds(), MATRIX_SIZE, off);
	FastLED.show();
	// put your setup code here, to run once:
	// uint16_t result = calibrate_silence();
	// getSampleSpeed();
	// strip.Begin();
	// for (int i = 0; i < MATRIX_WIDTH; i++) {
	// 	for (int j = 0; j < MATRIX_HEIGHT; j++) {
	// 		strip.SetPixelColor(tiles.Map(i, j), red);
	// 	}
	// 	strip.Show();
	// }
	// strip.ClearTo(RgbColor(0, 0, 0));
	// strip.Show();
	std::random_shuffle(animationTypes, std::end(animationTypes));
	chosenAnimation = pulseToBase;
	xTaskCreatePinnedToCore(
	    callMatrixFunction,
	    "Matrix rendering",
	    2048,
	    NULL,
	    10,
	    &MatrixTask,
	    0);
	analogWrite(fanPin, 512);
}

void loop() {
	getSamples(NSAMPLES);
	std::fill(imag, std::end(imag), 0.0);
	doFFT();
	int i = 2;
	double freq = (i * SAMPLE_RATE) / NSAMPLES;
	// while (freq < FREQ_THRESHOLD) {
	//     i++;
	//     freq = (i * SAMPLE_RATE) / NSAMPLES;
	// }
	if (DEBUG) {
		Serial.printf("Starting from: %lf\n", freq);
	}
	for (int bIndex = 0; bIndex < NBANDS; bIndex++) {
		const int band = BANDS[bIndex];
		double result = 0;
		int nbands = 0;
		while (freq < band) {
			freq = (i * SAMPLE_RATE) / NSAMPLES;
			result += real[i];
			nbands++;
			i++;
		}
		result /= nbands;
		// 220 - noise
		if (THRESHOLD > result) {
			result = 0.0;
		}
		bandValues[bIndex] = result;
		if (DEBUG) {
			Serial.printf("%d Hz: %lf\n", band, result);
		}
	}
	addAnimations = true;
	// for (int i = 0; i < NSAMPLES / 2; i++) {
	//     // Serial.printf("Peak: %lf\n", peak);
	//     Serial.printf("%f Hz: %lf\n", freq, real[i]);
	//     // Serial.printf("%f Hz: %i\n", FFT_BIN(i, SAMPLE_RATE, NSAMPLES), readings[i]);
	// }
	if (DEBUG) {
		Serial.printf("---------\n");
	}
}
