/*
 * NyxExpress firmware-owned WS2812 renderer.
 *
 * The module deliberately does not depend on LispBM or the native-library
 * ABI.  Configuration is exposed through the regular Express main config.
 * All frame storage is static and all input is range checked before it can
 * reach the renderer.
 */

#include "nyx_lighting.h"

#include "commands.h"
#include "comm_can.h"
#include "main.h"
#include "rgbled/lispif_rgbled_extensions.h"
#include "terminal.h"
#include "hwconf/nyx_conf_default.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "nvs.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define NYX_MAX_LEDS       200
#define NYX_FRAME_BYTES    (NYX_MAX_LEDS * 3)
#define NYX_APPDATA_MAX    220U
#define NYX_MAX_EFFECT     49
#define NYX_FRAME_MS       33U
#define NYX_KEEPALIVE_MS   2000U
#define NYX_BUILTIN_PALETTE_COUNT 18
#define NYX_PALETTE_CUSTOM        18
#define NYX_PALETTE_COUNT         19

typedef struct {
	uint32_t magic;
	uint32_t version;
	int32_t effect;
	int32_t led_count;
	float brightness;
	float speed;
	uint8_t red;
	uint8_t green;
	uint8_t blue;
	uint8_t auto_start;
	uint8_t reserved[4];
} nyx_legacy_persisted_t;

typedef struct {
	uint32_t magic;
	uint32_t version;
	int32_t can_id;
	float adc_threshold;
	float brightness;
	float blink_hz;
	uint8_t enabled;
	uint8_t active_high;
	uint8_t reserved[2];
} nyx_legacy_brake_t;

#define NYX_LEGACY_SETTINGS_MAGIC 0x4E59584CUL
#define NYX_LEGACY_BRAKE_MAGIC    0x4E594252UL

typedef struct {
	int pin;
	unsigned int timing;
	int effect;
	int palette;
	int color;
	int led_count;
	float brightness;
	float speed;
	uint8_t red;
	uint8_t green;
	uint8_t blue;
	bool running;
	bool auto_start;
	bool brake_enabled;
	int brake_can_id;
	float brake_adc_threshold;
	bool brake_active_high;
	float brake_brightness;
	float brake_blink_hz;
	int brake_color;
	uint8_t brake_red;
	uint8_t brake_green;
	uint8_t brake_blue;
	bool brake_active;
	bool initialized;
	uint32_t phase;
	uint32_t quiet_ms;
	uint32_t frame_hash;
	SemaphoreHandle_t mutex;
	StaticSemaphore_t mutex_buf;
	TaskHandle_t task;
	uint8_t frame[NYX_FRAME_BYTES];
	uint32_t work[NYX_MAX_LEDS];
} nyx_state_t;

static const uint32_t nyx_standard_colors[] = {
	0xFF0000, 0xFF4000, 0xFF8000, 0xFFFF00, 0x00FF00,
	0x00FFFF, 0x40C0FF, 0x0000FF, 0x8000FF, 0xFF00FF,
	0xFF80C0, 0xFFFFFF, 0xFFB060, 0x80FF00, 0xC0E8FF
};
#define NYX_STANDARD_COLOR_COUNT ((int)(sizeof(nyx_standard_colors) / sizeof(nyx_standard_colors[0])))
#define NYX_COLOR_CUSTOM 15

static nyx_state_t m_state;

static uint32_t nyx_pack(uint32_t r, uint32_t g, uint32_t b);

static int clamp_i(int v, int lo, int hi) {
	if (v < lo) return lo;
	if (v > hi) return hi;
	return v;
}

static float clamp_f(float v, float lo, float hi) {
	if (!isfinite(v)) return lo;
	if (v < lo) return lo;
	if (v > hi) return hi;
	return v;
}

static uint32_t nyx_selected_color(int preset, uint8_t r, uint8_t g, uint8_t b) {
	if (preset >= 0 && preset < NYX_STANDARD_COLOR_COUNT) return nyx_standard_colors[preset];
	return nyx_pack(r, g, b);
}

static void zero_bytes(uint8_t *data, size_t len) {
	memset(data, 0, len);
}

static bool nyx_legacy_settings_valid(const nyx_legacy_persisted_t *p) {
	return p->magic == NYX_LEGACY_SETTINGS_MAGIC && p->version == 1U &&
			p->effect >= 0 && p->effect < 50 &&
			p->led_count >= 1 && p->led_count <= NYX_MAX_LEDS &&
			isfinite(p->brightness) && p->brightness >= 0.0f && p->brightness <= 1.0f &&
			isfinite(p->speed) && p->speed >= 0.1f && p->speed <= 4.0f &&
			p->auto_start <= 1;
}

static bool nyx_legacy_brake_valid(const nyx_legacy_brake_t *p) {
	return p->magic == NYX_LEGACY_BRAKE_MAGIC && p->version == 1U &&
			p->can_id >= 0 && p->can_id <= 255 &&
			isfinite(p->adc_threshold) && p->adc_threshold >= 0.0f && p->adc_threshold <= 5.0f &&
			isfinite(p->brightness) && p->brightness >= 0.0f && p->brightness <= 1.0f &&
			isfinite(p->blink_hz) && p->blink_hz >= 0.5f && p->blink_hz <= 8.0f &&
			p->enabled <= 1 && p->active_high <= 1;
}

static void nyx_migrate_legacy_settings(void) {
	nvs_handle_t handle;
	if (nvs_open("nyxlight", NVS_READWRITE, &handle) != ESP_OK) return;

	uint8_t migrated = 0;
	if (nvs_get_u8(handle, "maincfg", &migrated) == ESP_OK && migrated) {
		nvs_close(handle);
		return;
	}

	nyx_legacy_persisted_t old_settings = {0};
	nyx_legacy_brake_t old_brake = {0};
	size_t settings_size = sizeof(old_settings);
	size_t brake_size = sizeof(old_brake);
	bool settings_ok = nvs_get_blob(handle, "settings", &old_settings, &settings_size) == ESP_OK &&
			settings_size == sizeof(old_settings) && nyx_legacy_settings_valid(&old_settings);
	bool brake_ok = nvs_get_blob(handle, "brake", &old_brake, &brake_size) == ESP_OK &&
			brake_size == sizeof(old_brake) && nyx_legacy_brake_valid(&old_brake);

	if (settings_ok) {
		backup.config.lighting_enabled = old_settings.auto_start;
		backup.config.lighting_effect = (uint8_t)old_settings.effect;
		backup.config.lighting_palette = NYX_PALETTE_CUSTOM;
		backup.config.lighting_color = NYX_COLOR_CUSTOM;
		backup.config.lighting_led_count = (uint16_t)old_settings.led_count;
		backup.config.lighting_brightness = old_settings.brightness;
		backup.config.lighting_speed = old_settings.speed;
		backup.config.lighting_auto_start = old_settings.auto_start;
		backup.config.lighting_red = old_settings.red;
		backup.config.lighting_green = old_settings.green;
		backup.config.lighting_blue = old_settings.blue;
	}
	if (brake_ok) {
		backup.config.brake_enabled = old_brake.enabled;
		backup.config.brake_can_id = (uint16_t)old_brake.can_id;
		backup.config.brake_adc_threshold = old_brake.adc_threshold;
		backup.config.brake_active_high = old_brake.active_high;
		backup.config.brake_brightness = old_brake.brightness;
		backup.config.brake_blink_hz = old_brake.blink_hz;
		backup.config.brake_color = NYX_COLOR_CUSTOM;
		backup.config.brake_red = 255;
		backup.config.brake_green = 255;
		backup.config.brake_blue = 0;
	}

	nvs_set_u8(handle, "maincfg", 1);
	nvs_commit(handle);
	nvs_close(handle);
	if (settings_ok || brake_ok) main_store_backup_data();
}

static uint8_t brightness8(const nyx_state_t *s) {
	return (uint8_t)(clamp_f(s->brightness, 0.0f, 1.0f) * 255.0f + 0.5f);
}

static uint32_t nyx_pack(uint32_t r, uint32_t g, uint32_t b) {
	return (r << 16) | (g << 8) | b;
}

static uint32_t nyx_scale(uint32_t c, uint32_t level) {
	return nyx_pack(((c >> 16) & 0xFFU) * level / 255U,
			((c >> 8) & 0xFFU) * level / 255U,
			(c & 0xFFU) * level / 255U);
}

static const uint32_t nyx_palettes[][4] = {
	{0xFF0000, 0x00FF00, 0x0000FF, 0xFFFFFF},
	{0xFF0000, 0xFF8000, 0xFFFF00, 0xFF0000},
	{0x0000FF, 0x00FFFF, 0x00FF80, 0x0000FF},
	{0xFF00FF, 0x8000FF, 0x0080FF, 0xFF00FF},
	{0xFFFFFF, 0xFF4000, 0x400000, 0x000000},
	{0x00FF00, 0xFFFF00, 0xFF0000, 0x00FF00},
	{0xFFFFFF, 0x000000, 0xFFFFFF, 0x000000},
	{0x1030FF, 0xFFFFFF, 0x1030FF, 0x001040},
	{0x0B1D51, 0xFF6A00, 0xFFD700, 0x2A0E4F},
	{0x000000, 0x8B0000, 0xFF4500, 0xFFFFE0},
	{0x01411F, 0x00FFB2, 0x7A00FF, 0x013220},
	{0x013220, 0x2E8B57, 0x9ACD32, 0x013220},
	{0x8000FF, 0xFF4000, 0xFF00A0, 0x0040FF},
	{0x001F5C, 0x00BFFF, 0xFFFFFF, 0x001F5C},
	{0xFF6A00, 0x1A001A, 0x8000FF, 0x000000},
	{0xFF0000, 0x00FF00, 0xFFC000, 0x0000FF},
	{0xFFB3BA, 0xBAFFC9, 0xBAE1FF, 0xFFFFBA},
	{0xFF2E6A, 0xFFC0CB, 0xFFF0F5, 0xC71585}
};

static uint32_t nyx_palette_at(int palette_id, uint8_t pos) {
	const uint32_t *p = nyx_palettes[palette_id % NYX_BUILTIN_PALETTE_COUNT];
	uint32_t a = p[pos / 64U];
	uint32_t b = p[(pos / 64U + 1U) & 3U];
	uint32_t f = pos & 63U;
	return nyx_pack(
			(((a >> 16) & 0xFFU) * (63U - f) + ((b >> 16) & 0xFFU) * f) / 63U,
			(((a >> 8) & 0xFFU) * (63U - f) + ((b >> 8) & 0xFFU) * f) / 63U,
			((a & 0xFFU) * (63U - f) + (b & 0xFFU) * f) / 63U);
}

static uint32_t nyx_palette_for_state(const nyx_state_t *s, int palette_id,
		uint8_t pos) {
	if (palette_id == NYX_PALETTE_CUSTOM) {
		return nyx_pack(s->red, s->green, s->blue);
	}
	return nyx_palette_at(palette_id, pos);
}

static uint32_t triangle(uint32_t x) {
	x &= 511U;
	return x < 256U ? x : 511U - x;
}

static void nyx_fill_work(nyx_state_t *s, uint32_t color) {
	for (int i = 0; i < s->led_count; i++) s->work[i] = color;
}

/* Keep the existing 50-item UI stable, but route it through the compact
 * integer-only renderer families introduced by lib_esp_led_strip. */
static int nyx_effect_renderer_id(int effect) {
	switch (effect) {
	case 0: return 0;  /* static */
	case 1: return 3;  /* rainbow */
	case 2: return 1;  /* breathing */
	case 3: return 5;  /* comet */
	case 4: return 11; /* wipe */
	case 5: return 9;  /* police */
	case 6: return 15; /* fire */
	case 7: return 8;  /* scanner */
	case 8: return 10; /* theater */
	case 9: return 4;  /* twinkle */
	case 10: return 5; /* meteor/comet */
	case 11: return 12; /* palette wave */
	case 12: return 7; /* blink/strobe */
	case 13: return 3; /* colorloop/rainbow */
	case 14: return 2; /* chase rainbow/chase */
	case 15: return 2; /* running/chase */
	case 16: return 12; /* saw/waves */
	case 17: case 18: return 4; /* dissolve/sparkle */
	case 19: case 20: return 7; /* strobe/lightning */
	case 21: return 9; /* traffic light/police */
	case 22: case 23: case 24: case 25: return 12; /* seasonal palettes */
	case 26: case 27: case 28: return 8; /* dots/ripple/scanners */
	case 29: case 33: case 34: case 42: case 45: case 47: case 48: case 49:
		return 12; /* gradients and waves */
	case 30: case 44: case 46: return 15; /* matrix/fire variants */
	case 31: case 36: case 39: return 4; /* sparkle variants */
	case 32: return 2; /* sinelon */
	case 35: return 3; /* rainbow runner */
	case 37: case 38: return 8; /* cylon/larson */
	case 40: return 11; /* random wipe */
	case 41: return 5; /* smooth meteor */
	case 43: return 1; /* in-out/breathe */
	default: return 0;
	}
}

static void nyx_render_work(nyx_state_t *s) {
	int n = s->led_count;
	uint32_t ph = s->phase;
	uint32_t color = nyx_selected_color(s->color, s->red, s->green, s->blue);
	int effect = nyx_effect_renderer_id(s->effect);
	int palette_id = clamp_i(s->palette, 0, NYX_PALETTE_COUNT - 1);
	memset(s->work, 0, sizeof(s->work));

	switch (effect) {
	case 0: // FX_SOLID
		nyx_fill_work(s, color);
		break;
	case 1: { // FX_BREATHE
		uint32_t c = nyx_scale(color, triangle(ph / 4U));
		nyx_fill_work(s, c);
		break;
	}
	case 2: { // FX_CHASE
		int head = (int)((ph / 32U) % (uint32_t)n);
		for (int i = 0; i < n; i++) {
			int d = i - head;
			if (d < 0) d += n;
			s->work[i] = d < 8 ? nyx_scale(color, 255U - (uint32_t)d * 255U / 8U) : 0;
		}
		break;
	}
	case 3: // FX_RAINBOW
		for (int i = 0; i < n; i++) s->work[i] = nyx_palette_for_state(s, palette_id,
				(uint8_t)((i * 255 / n + ph / 16U) & 255U));
		break;
	case 4: { // FX_SPARKLE
		uint32_t step = ph / 128U;
		uint32_t threshold = 12U + (uint32_t)(clamp_f(s->speed, 0.1f, 4.0f) * 5.0f);
		for (int i = 0; i < n; i++) {
			uint32_t h = ((uint32_t)i * 2654435761U) ^ (step * 40503U);
			s->work[i] = ((h >> 8) & 0xFFU) < threshold ? color : 0;
		}
		break;
	}
	case 5: { // FX_COMET
		int head = (int)((ph / 32U) % (uint32_t)n);
		for (int i = 0; i < n; i++) {
			int d = head - i;
			if (d < 0) d += n;
			s->work[i] = d < 8 ? nyx_scale(color, 255U - (uint32_t)d * 255U / 8U) : 0;
		}
		break;
	}
	case 6: { // FX_GAUGE, animated because the current UI has no fx_val
		uint32_t level = triangle(ph / 4U);
		int lit = (n * (int)level + 254) / 255;
		for (int i = 0; i < n; i++) {
			uint32_t c = nyx_palette_for_state(s, palette_id, (uint8_t)(i * 255 / n));
			s->work[i] = i < lit ? c : 0;
		}
		break;
	}
	case 7: { // FX_STROBE
		uint32_t flash = ph / 64U;
		if (!(flash & 1U)) nyx_fill_work(s, color);
		break;
	}
	case 8: { // FX_LARSON
		int span = n > 1 ? n - 1 : 1;
		int pos = (int)((ph / 16U) % (uint32_t)(2 * span));
		if (pos > span) pos = 2 * span - pos;
		for (int i = 0; i < n; i++) {
			int d = i > pos ? i - pos : pos - i;
			s->work[i] = d < 8 ? nyx_scale(color, 255U - (uint32_t)d * 255U / 8U) : 0;
		}
		break;
	}
	case 9: { // FX_FELONY / police
		bool swap = ((ph / 48U) & 1U) != 0;
		uint32_t left = swap ? 0x0000FFU : 0xFF0000U;
		uint32_t right = swap ? 0xFF0000U : 0x0000FFU;
		for (int i = 0; i < n; i++) s->work[i] = i < n / 2 ? left : right;
		break;
	}
	case 10: // FX_THEATER
		for (int i = 0; i < n; i++) {
			if (((i + 3 - (int)(ph / 64U) % 3) % 3) == 0) s->work[i] = color;
		}
		break;
	case 11: { // FX_WIPE
		uint32_t period = 2U * (uint32_t)n;
		uint32_t pos = (ph / 32U) % period;
		bool filling = pos < (uint32_t)n;
		int edge = (int)(filling ? pos : pos - (uint32_t)n);
		for (int i = 0; i < n; i++) {
			bool lit = filling ? i <= edge : i > edge;
			s->work[i] = lit ? color : 0;
		}
		break;
	}
	case 12: // FX_WAVES
		for (int i = 0; i < n; i++) {
			uint32_t p = (uint32_t)i * 255U / (uint32_t)n;
			uint32_t w1 = triangle(p * 2U + ph / 8U);
			uint32_t w2 = triangle(p * 3U + 170U + 1024U - ph / 12U);
			uint32_t w3 = triangle(p + 85U + ph / 20U);
			uint32_t c = nyx_palette_for_state(s, palette_id, (uint8_t)((w1 + w3) / 2U));
			s->work[i] = nyx_scale(c, 64U + w2 * 191U / 255U);
		}
		break;
	case 13: { // FX_CANDLE
		uint32_t t = ph / 128U;
		uint32_t f = (ph & 127U) * 2U;
		uint32_t flame = color ? color : 0xFF9329U;
		for (int i = 0; i < n; i++) {
			uint32_t h1 = ((uint32_t)i * 2654435761U) ^ (t * 40503U);
			uint32_t h2 = ((uint32_t)i * 2654435761U) ^ ((t + 1U) * 40503U);
			uint32_t b1 = (h1 >> 8) & 0xFFU;
			uint32_t b2 = (h2 >> 8) & 0xFFU;
			uint32_t b = (b1 * (255U - f) + b2 * f) / 255U;
			s->work[i] = nyx_scale(flame, 100U + b * 155U / 255U);
		}
		break;
	}
	case 14: { // FX_HEARTBEAT
		uint32_t cyc = (ph / 2U) & 511U;
		uint32_t level = 0;
		if (cyc < 96U) level = 255U - cyc * 255U / 96U;
		else if (cyc >= 160U && cyc < 240U) level = 180U - (cyc - 160U) * 180U / 80U;
		if (level < 16U) level = 16U;
		nyx_fill_work(s, nyx_scale(color, level));
		break;
	}
	case 15: { // compact fire, retained from the previous renderer
		for (int i = 0; i < n; i++) {
			uint8_t v = (uint8_t)((i * 17U + ph * 3U) % 100U);
			s->work[i] = nyx_pack(90U + v * 2U / 3U, v * 55U / 100U, v / 50U);
		}
		break;
	}
	case 16: // FX_OFF
	default:
		break;
	}
}

static void render_brake(nyx_state_t *s) {
	float blink_hz = clamp_f(s->brake_blink_hz, 0.5f, 8.0f);
	uint32_t half_period_ms = (uint32_t)(500.0f / blink_hz);
	if (half_period_ms < 20U) half_period_ms = 20U;
	TickType_t half_period_ticks = pdMS_TO_TICKS(half_period_ms);
	if (half_period_ticks == 0) half_period_ticks = 1;
	bool on = ((xTaskGetTickCount() / half_period_ticks) & 1U) != 0;
	uint8_t level = on ? (uint8_t)(clamp_f(s->brake_brightness, 0.0f, 1.0f) * 255.0f + 0.5f) : 0;
	uint32_t color = nyx_selected_color(s->brake_color,
			s->brake_red, s->brake_green, s->brake_blue);
	nyx_fill_work(s, nyx_scale(color, level));
}

static uint32_t nyx_hash_frame(const uint8_t *data, size_t len) {
	uint32_t hash = 2166136261U;
	for (size_t i = 0; i < len; i++) hash = (hash ^ data[i]) * 16777619U;
	return hash;
}

static void render_frame(nyx_state_t *s) {
	if (s->brake_active) render_brake(s);
	else nyx_render_work(s);

	uint8_t level = brightness8(s);
	for (int i = 0; i < s->led_count; i++) {
		uint32_t c = nyx_scale(s->work[i], level);
		int off = i * 3;
		/* The Express RGB driver consumes GRB bytes. */
		s->frame[off + 0] = (uint8_t)((c >> 8) & 0xFFU);
		s->frame[off + 1] = (uint8_t)((c >> 16) & 0xFFU);
		s->frame[off + 2] = (uint8_t)(c & 0xFFU);
	}
}

static void nyx_send_text(const char *text) {
	if (!text) return;
	size_t len = strlen(text);
	if (len == 0 || len >= NYX_APPDATA_MAX) return;
	commands_send_app_data((unsigned char *)text, (unsigned int)len);
}

static void nyx_send_message(const char *message) {
	char text[NYX_APPDATA_MAX];
	int len = snprintf(text, sizeof(text), "lighting msg %s", message ? message : "");
	if (len > 0 && (size_t)len < sizeof(text)) nyx_send_text(text);
}

static void nyx_send_settings(void) {
	nyx_state_t snapshot;
	memset(&snapshot, 0, sizeof(snapshot));
	xSemaphoreTake(m_state.mutex, portMAX_DELAY);
	snapshot.effect = m_state.effect;
	snapshot.palette = m_state.palette;
	snapshot.led_count = m_state.led_count;
	snapshot.brightness = m_state.brightness;
	snapshot.speed = m_state.speed;
	snapshot.red = m_state.red;
	snapshot.green = m_state.green;
	snapshot.blue = m_state.blue;
	snapshot.running = m_state.running;
	snapshot.auto_start = m_state.auto_start;
	snapshot.brake_enabled = m_state.brake_enabled;
	snapshot.brake_can_id = m_state.brake_can_id;
	snapshot.brake_adc_threshold = m_state.brake_adc_threshold;
	snapshot.brake_active_high = m_state.brake_active_high;
	snapshot.brake_brightness = m_state.brake_brightness;
	snapshot.brake_blink_hz = m_state.brake_blink_hz;
	snapshot.initialized = m_state.initialized;
	xSemaphoreGive(m_state.mutex);

	char text[NYX_APPDATA_MAX];
	int len = snprintf(text, sizeof(text),
			"lighting settings %d %d %.3f %.3f %d %d %d %d %d %d",
		snapshot.effect, snapshot.led_count, snapshot.brightness,
			 snapshot.speed, snapshot.red, snapshot.green, snapshot.blue,
			snapshot.running ? 1 : 0, snapshot.auto_start ? 1 : 0,
			snapshot.initialized ? 1 : 0);
	if (len > 0 && (size_t)len < sizeof(text)) nyx_send_text(text);
}

static void nyx_turn_off_locked(nyx_state_t *s) {
	s->running = false;
	s->brake_active = false;
	zero_bytes(s->frame, sizeof(s->frame));
	if (s->initialized) {
		rgbled_update(s->pin, s->frame, (size_t)s->led_count * 3U);
	}
}

static void nyx_apply_main_config_locked(nyx_state_t *s, bool boot) {
	const main_config_t *cfg = (const main_config_t *)&backup.config;
	s->effect = clamp_i(cfg->lighting_effect, 0, NYX_MAX_EFFECT);
	s->palette = clamp_i(cfg->lighting_palette, 0, NYX_PALETTE_COUNT - 1);
	s->color = clamp_i(cfg->lighting_color, 0, NYX_COLOR_CUSTOM);
	s->led_count = clamp_i(cfg->lighting_led_count, 1, NYX_MAX_LEDS);
	s->brightness = clamp_f(cfg->lighting_brightness, 0.0f, 1.0f);
	s->speed = clamp_f(cfg->lighting_speed, 0.1f, 4.0f);
	s->red = (uint8_t)clamp_i(cfg->lighting_red, 0, 255);
	s->green = (uint8_t)clamp_i(cfg->lighting_green, 0, 255);
	s->blue = (uint8_t)clamp_i(cfg->lighting_blue, 0, 255);
	s->auto_start = cfg->lighting_auto_start != 0;
	s->brake_enabled = cfg->brake_enabled != 0;
	s->brake_can_id = clamp_i(cfg->brake_can_id, 0, 255);
	s->brake_adc_threshold = clamp_f(cfg->brake_adc_threshold, 0.0f, 5.0f);
	s->brake_active_high = cfg->brake_active_high != 0;
	s->brake_brightness = clamp_f(cfg->brake_brightness, 0.0f, 1.0f);
	s->brake_blink_hz = clamp_f(cfg->brake_blink_hz, 0.5f, 8.0f);
	s->brake_color = clamp_i(cfg->brake_color, 0, NYX_COLOR_CUSTOM);
	s->brake_red = (uint8_t)clamp_i(cfg->brake_red, 0, 255);
	s->brake_green = (uint8_t)clamp_i(cfg->brake_green, 0, 255);
	s->brake_blue = (uint8_t)clamp_i(cfg->brake_blue, 0, 255);
	s->brake_active = false;
	s->phase = 0;
	s->quiet_ms = NYX_KEEPALIVE_MS;
	s->frame_hash = 0;
	s->running = cfg->lighting_enabled != 0 && (!boot || s->auto_start);
	if (!s->running) nyx_turn_off_locked(s);
}

static void nyx_apply_locked(nyx_state_t *s, int effect, int led_count,
		float brightness, float speed, int red, int green, int blue, int running) {
	s->effect = clamp_i(effect, 0, NYX_MAX_EFFECT);
	s->led_count = clamp_i(led_count, 1, NYX_MAX_LEDS);
	s->brightness = clamp_f(brightness, 0.0f, 1.0f);
	s->speed = clamp_f(speed, 0.1f, 4.0f);
	s->red = (uint8_t)clamp_i(red, 0, 255);
	s->green = (uint8_t)clamp_i(green, 0, 255);
	s->blue = (uint8_t)clamp_i(blue, 0, 255);
	s->running = running != 0;
	s->phase = 0;
	s->quiet_ms = NYX_KEEPALIVE_MS;
	s->frame_hash = 0;
	if (!s->running) nyx_turn_off_locked(s);
}

static void nyx_app_data_handler(unsigned char *data, unsigned int len) {
	if (!data || len == 0 || len >= NYX_APPDATA_MAX) return;
	char cmd[NYX_APPDATA_MAX];
	memcpy(cmd, data, len);
	cmd[len] = '\0';
	while (len > 0 && (cmd[len - 1] == '\0' || cmd[len - 1] == '\r' ||
			cmd[len - 1] == '\n' || cmd[len - 1] == ' ' || cmd[len - 1] == '\t')) {
		cmd[--len] = '\0';
	}

	if (strcmp(cmd, "nyxlight get") == 0) {
		nyx_send_settings();
		return;
	}

	int effect, led_count, red, green, blue, running, auto_start;
	float brightness, speed;
	int parsed = sscanf(cmd, "nyxlight set %d %d %f %f %d %d %d %d %d",
			&effect, &led_count, &brightness, &speed, &red, &green, &blue,
			&running, &auto_start);
	if (parsed != 9) {
		parsed = sscanf(cmd, "nyxlight save %d %d %f %f %d %d %d %d %d",
				&effect, &led_count, &brightness, &speed, &red, &green, &blue,
				&running, &auto_start);
	}
	if (parsed == 9) {
		xSemaphoreTake(m_state.mutex, portMAX_DELAY);
		m_state.auto_start = auto_start != 0;
		nyx_apply_locked(&m_state, effect, led_count, brightness, speed,
				red, green, blue, running);
		xSemaphoreGive(m_state.mutex);
		nyx_send_settings();
		return;
	}

	int value;
	if (sscanf(cmd, "nyxlight run %d", &value) == 1) {
		xSemaphoreTake(m_state.mutex, portMAX_DELAY);
		if (value) m_state.running = true;
		else nyx_turn_off_locked(&m_state);
		xSemaphoreGive(m_state.mutex);
		nyx_send_settings();
		return;
	}

	if (strcmp(cmd, "nyxlight off") == 0) {
		xSemaphoreTake(m_state.mutex, portMAX_DELAY);
		nyx_turn_off_locked(&m_state);
		xSemaphoreGive(m_state.mutex);
		nyx_send_settings();
		return;
	}

	nyx_send_message("unknown command");
}

static void terminal_nyxlight(int argc, const char **argv) {
	if (argc > 1 && strcmp(argv[1], "off") == 0) {
		xSemaphoreTake(m_state.mutex, portMAX_DELAY);
		nyx_turn_off_locked(&m_state);
		xSemaphoreGive(m_state.mutex);
	} else if (argc > 1 && strcmp(argv[1], "on") == 0) {
		xSemaphoreTake(m_state.mutex, portMAX_DELAY);
		m_state.running = true;
		xSemaphoreGive(m_state.mutex);
	} else {
		commands_printf("NyxLighting: initialized=%d pin=%d effect=%d leds=%d brightness=%.2f speed=%.2f running=%d auto=%d brake=%d can=%d adc-th=%.2f active=%d",
				m_state.initialized, m_state.pin, m_state.effect, m_state.led_count,
				m_state.brightness, m_state.speed, m_state.running, m_state.auto_start,
				m_state.brake_enabled, m_state.brake_can_id, m_state.brake_adc_threshold,
				m_state.brake_active);
	}
}

static void nyx_update_brake_locked(nyx_state_t *s) {
	bool active = false;
	if (s->brake_enabled) {
		can_status_msg_6 *status = comm_can_get_status_msg_6_id(s->brake_can_id);
		if (status) {
			uint32_t now = xTaskGetTickCount();
			uint32_t stale_ticks = pdMS_TO_TICKS(500);
			if ((uint32_t)(now - status->rx_time) <= stale_ticks) {
				float threshold = clamp_f(s->brake_adc_threshold, 0.0f, 5.0f);
				float hysteresis = 0.08f;
				if (s->brake_active_high) {
					active = s->brake_active ? status->adc_2 >= threshold - hysteresis :
							status->adc_2 >= threshold;
				} else {
					active = s->brake_active ? status->adc_2 <= threshold + hysteresis :
							status->adc_2 <= threshold;
				}
			}
		}
	}
	s->brake_active = active;
}

static void nyx_render_task(void *arg) {
	nyx_state_t *s = (nyx_state_t *)arg;
	TickType_t previous_tick = xTaskGetTickCount();
	for (;;) {
		TickType_t now = xTaskGetTickCount();
		uint32_t elapsed_ms = (uint32_t)pdTICKS_TO_MS(now - previous_tick);
		previous_tick = now;
		if (elapsed_ms > 250U) elapsed_ms = 250U;
		xSemaphoreTake(s->mutex, portMAX_DELAY);
		if (s->initialized && s->running) {
			nyx_update_brake_locked(s);
			uint32_t speed_phase = (uint32_t)(clamp_f(s->speed, 0.1f, 4.0f) * 32.0f + 0.5f);
			s->phase += speed_phase * elapsed_ms / NYX_FRAME_MS;
			render_frame(s);
			uint32_t frame_len = (uint32_t)s->led_count * 3U;
			uint32_t hash = nyx_hash_frame(s->frame, frame_len);
			s->quiet_ms += elapsed_ms;
			if (hash != s->frame_hash || s->quiet_ms >= NYX_KEEPALIVE_MS) {
				rgbled_update(s->pin, s->frame, frame_len);
				s->frame_hash = hash;
				s->quiet_ms = 0;
			}
		}
		float speed = clamp_f(s->speed, 0.1f, 4.0f);
		xSemaphoreGive(s->mutex);
		uint32_t delay_ms = (uint32_t)(NYX_FRAME_MS / speed);
		if (delay_ms < 5U) delay_ms = 5U;
		vTaskDelay(pdMS_TO_TICKS(delay_ms));
	}
}

void nyx_lighting_init(int pin, unsigned int timing_preset) {
	memset(&m_state, 0, sizeof(m_state));
	m_state.pin = pin;
	m_state.timing = timing_preset;
	m_state.mutex = xSemaphoreCreateMutexStatic(&m_state.mutex_buf);
	nyx_migrate_legacy_settings();

	xSemaphoreTake(m_state.mutex, portMAX_DELAY);
	nyx_apply_main_config_locked(&m_state, true);
	xSemaphoreGive(m_state.mutex);

	commands_set_app_data_handler(nyx_app_data_handler);
	terminal_register_command_callback("nyxlight", "NyxExpress WS2812 status", "[on|off]", terminal_nyxlight);

	if (!rgbled_init(m_state.pin, m_state.timing)) {
		m_state.initialized = false;
		m_state.running = false;
		commands_printf("NyxLighting: RGB init failed on GPIO%d", m_state.pin);
		return;
	}

	m_state.initialized = true;
	if (xTaskCreate(nyx_render_task, "nyx_light", 3072, &m_state,
			configMAX_PRIORITIES - 3, &m_state.task) != pdPASS) {
		m_state.initialized = false;
		m_state.running = false;
		zero_bytes(m_state.frame, sizeof(m_state.frame));
		rgbled_update(m_state.pin, m_state.frame, (size_t)m_state.led_count * 3U);
		commands_printf("NyxLighting: renderer task creation failed");
		return;
	}

	commands_printf("NyxLighting: native renderer ready, GPIO%d, %d LEDs, effect %d",
			m_state.pin, m_state.led_count, m_state.effect);
}

void nyx_lighting_config_changed(void) {
	if (!m_state.mutex) return;
	xSemaphoreTake(m_state.mutex, portMAX_DELAY);
	nyx_apply_main_config_locked(&m_state, false);
	xSemaphoreGive(m_state.mutex);
}
