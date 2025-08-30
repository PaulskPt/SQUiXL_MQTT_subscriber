#include <vector>
#include <utils/lodepng.h>
#include <LittleFS.h>
#include "bb_spi_lcd.h"
#include "settings/settings_async.h"

#include <cmath>
#include <algorithm>
#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <atomic>
#include "screenie.h"

//-------------------- HELPERS --------------------
inline uint8_t percent_to_255(float pct)
{
	return uint8_t(std::clamp(pct, 0.0f, 100.0f) * 255.0f / 100.0f + 0.5f);
}

inline uint8_t norm_to_255(float norm)
{
	return uint8_t(std::clamp(norm, 0.0f, 1.0f) * 255.0f + 0.5f);
}

uint8_t apply_levels(uint8_t input, uint8_t in_black, uint8_t in_white, float gamma, uint8_t out_black, uint8_t out_white)
{
	float clamped = std::clamp(float(input), float(in_black), float(in_white));
	float normalized = (clamped - float(in_black)) / float(in_white - in_black);
	normalized = std::clamp(normalized, 0.0f, 1.0f);
	float gamma_corrected = std::pow(normalized, gamma);
	float output = float(out_black) + gamma_corrected * float(out_white - out_black);
	return static_cast<uint8_t>(std::clamp(output, 0.0f, 255.0f));
}

uint8_t adjust_contrast(uint8_t val, float contrast)
{
	int out = int((float(val) - 128.0f) * contrast + 128.0f + 0.5f);
	return std::clamp(out, 0, 255);
}

void rgb_to_hsl(uint8_t r, uint8_t g, uint8_t b, float &h, float &s, float &l)
{
	float rf = r / 255.0f, gf = g / 255.0f, bf = b / 255.0f;
	float max = std::max(rf, std::max(gf, bf)), min = std::min(rf, std::min(gf, bf));
	l = (max + min) / 2.0f;
	if (max == min)
	{
		h = s = 0.0f;
	}
	else
	{
		float d = max - min;
		s = (l > 0.5f) ? d / (2.0f - max - min) : d / (max + min);
		if (max == rf)
			h = (gf - bf) / d + (gf < bf ? 6.0f : 0.0f);
		else if (max == gf)
			h = (bf - rf) / d + 2.0f;
		else
			h = (rf - gf) / d + 4.0f;
		h /= 6.0f;
	}
}

void hsl_to_rgb(float h, float s, float l, uint8_t &r, uint8_t &g, uint8_t &b)
{
	auto hue2rgb = [](float p, float q, float t) {
		if (t < 0.0f)
			t += 1.0f;
		if (t > 1.0f)
			t -= 1.0f;
		if (t < 1.0f / 6.0f)
			return p + (q - p) * 6.0f * t;
		if (t < 1.0f / 2.0f)
			return q;
		if (t < 2.0f / 3.0f)
			return p + (q - p) * (2.0f / 3.0f - t) * 6.0f;
		return p;
	};
	float r1, g1, b1;
	if (s == 0)
		r1 = g1 = b1 = l;
	else
	{
		float q = l < 0.5f ? l * (1.0f + s) : l + s - l * s;
		float p = 2.0f * l - q;
		r1 = hue2rgb(p, q, h + 1.0f / 3.0f);
		g1 = hue2rgb(p, q, h);
		b1 = hue2rgb(p, q, h - 1.0f / 3.0f);
	}
	r = uint8_t(std::clamp(r1 * 255.0f, 0.0f, 255.0f));
	g = uint8_t(std::clamp(g1 * 255.0f, 0.0f, 255.0f));
	b = uint8_t(std::clamp(b1 * 255.0f, 0.0f, 255.0f));
}

void adjust_saturation(uint8_t &r, uint8_t &g, uint8_t &b, float sat_factor)
{
	float h, s, l;
	rgb_to_hsl(r, g, b, h, s, l);
	s *= sat_factor;
	s = std::clamp(s, 0.0f, 1.0f);
	hsl_to_rgb(h, s, l, r, g, b);
}

void apply_affinity_style_white_balance(uint8_t &r, uint8_t &g, uint8_t &b, float temp, float tint)
{
	float red = r;
	float green = g;
	float blue = b;

	float temp_strength = 60.0f;
	float tint_strength = 40.0f;

	red += temp * temp_strength;
	blue -= temp * temp_strength;

	green -= tint * tint_strength;
	red += tint * (tint_strength / 2.0f);
	blue += tint * (tint_strength / 2.0f);

	r = std::clamp(int(red), 0, 255);
	g = std::clamp(int(green), 0, 255);
	b = std::clamp(int(blue), 0, 255);
}

//-------------------- TASK MANAGEMENT --------------------

struct ScreenshotTaskArgs
{
		BB_SPI_LCD *screen;
		ScreenshotCallback callback;
};

std::atomic<bool> screenshot_task_running{false};

void save_png_task(void *pvParameters)
{
	ScreenshotTaskArgs *args = (ScreenshotTaskArgs *)pvParameters;
	BB_SPI_LCD *screen = args->screen;
	ScreenshotCallback callback = std::move(args->callback);
	delete args;

	bool success = false;

	if (!LittleFS.begin())
	{
		Serial.println("❌ LittleFS mount failed");
		if (callback)
			callback(false);
		screenshot_task_running = false;
		vTaskDelete(NULL);
		return;
	}

	const uint8_t *raw = reinterpret_cast<const uint8_t *>(screen->getBuffer());
	const uint32_t W = 480, H = 480;
	size_t outIdx = 0;
	size_t pxCount = (size_t)W * H;
	auto rgb = (uint8_t *)heap_caps_malloc(
		pxCount * 3,
		MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
	);

	if (!rgb)
	{
		Serial.println("❌ Failed to allocate RGB buffer");
		if (callback)
			callback(false);
		screenshot_task_running = false;
		vTaskDelete(NULL);
		return;
	}

	uint8_t black = norm_to_255(settings.config.screenshot.black);
	uint8_t white = norm_to_255(settings.config.screenshot.white);

	for (uint32_t y = 0; y < H; y++)
	{
		for (uint32_t x = 0; x < W; x++)
		{
			size_t inIdx = (y * W + x) * 2;
			uint16_t pix565 = (uint16_t(raw[inIdx]) << 8) | raw[inIdx + 1];

			uint8_t r = ((pix565 >> 11) & 0x1F);
			r = (r << 3) | (r >> 2);

			uint8_t g = ((pix565 >> 5) & 0x3F);
			g = (g << 2) | (g >> 4);

			uint8_t b = (pix565 & 0x1F);
			b = (b << 3) | (b >> 2);

			if (settings.config.screenshot.gamma != 1.0 || settings.config.screenshot.black > 0.0 || settings.config.screenshot.white < 1.0)
			{
				r = apply_levels(r, black, white, settings.config.screenshot.gamma, 0, 255);
				g = apply_levels(g, black, white, settings.config.screenshot.gamma, 0, 255);
				b = apply_levels(b, black, white, settings.config.screenshot.gamma, 0, 255);
			}

			if (settings.config.screenshot.saturation != 1.0)
			{
				adjust_saturation(r, g, b, settings.config.screenshot.saturation);
			}

			if (settings.config.screenshot.contrast != 1.0)
			{
				r = adjust_contrast(r, settings.config.screenshot.contrast);
				g = adjust_contrast(g, settings.config.screenshot.contrast);
				b = adjust_contrast(b, settings.config.screenshot.contrast);
			}

			if (settings.config.screenshot.temperature != 1.0 || settings.config.screenshot.tint != 1.0)
			{
				apply_affinity_style_white_balance(r, g, b, settings.config.screenshot.temperature, settings.config.screenshot.tint);
			}

			rgb[outIdx++] = r;
			rgb[outIdx++] = g;
			rgb[outIdx++] = b;
		}
	}

	std::vector<uint8_t> png;
	unsigned err = lodepng::encode(
		png,
		rgb,
		W, H,
		LCT_RGB,
		8
	);

	heap_caps_free(rgb);
	rgb = nullptr;

	if (err)
	{
		Serial.printf("❌ PNG encode failed (%u): %s\n", err, lodepng_error_text(err));
		if (callback)
			callback(false);
		screenshot_task_running = false;
		vTaskDelete(NULL);
		return;
	}

	File f = LittleFS.open("/screenshot.png", FILE_WRITE);
	if (!f)
	{
		Serial.println("❌ Failed to open /screenshot.png for write");
		if (callback)
			callback(false);
		screenshot_task_running = false;
		vTaskDelete(NULL);
		return;
	}
	f.write(png.data(), png.size());
	f.close();

	Serial.printf("✅ Wrote screenshot.png (%u bytes)\n", (unsigned)png.size());
	if (callback)
		callback(true);
	screenshot_task_running = false;
	vTaskDelete(NULL);
}

// Call this from your main loop or UI task to request a screenshot
// Returns true if the task started, false if one is already running
bool request_screenshot(BB_SPI_LCD *screen, ScreenshotCallback cb)
{
	if (screenshot_task_running)
		return false; // already running

	screenshot_task_running = true;

	Serial.printf("\nHeap Log: SCREENIE\nHeap Size: %u of %u\n", ESP.getFreeHeap(), ESP.getHeapSize());
	Serial.printf("Min Heap Size: %u, Max Alloc Heap Size: %u, ", ESP.getMinFreeHeap(), ESP.getMaxAllocHeap());
	Serial.printf("PSRAM Free: %u\n", ESP.getFreePsram());
	Serial.printf("Largest PSRAM Chunk Free %u\n\n", heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));

	ScreenshotTaskArgs *args = new ScreenshotTaskArgs;
	args->screen = screen;
	args->callback = std::move(cb);

	BaseType_t res = xTaskCreatePinnedToCore(
		save_png_task,
		"SavePNG",
		14000, // Stack size
		args,
		3, // Priority
		nullptr,
		0 // Core 0
	);

	if (res != pdPASS)
	{
		Serial.println("❌ Failed to start screenshot task");
		screenshot_task_running = false;
		delete args;
		return false;
	}
	return true;
}