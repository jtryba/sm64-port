#if defined(TARGET_XBOX)

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <hal/xbox.h>
#include <SDL.h>
#include <SDL_raphnet.h>

#include <ultra64.h>

#include "controller_api.h"
#include "raphnet-n64-port-map-v1.h"

#define STICK_DEADZONE 8000
#define STICK_FULL_SCALE 0x4000
#define C_STICK_DEADZONE 0x4000
#define BUTTON_DEADZONE 0x20

/*
 * nxdk SDL converts an Xbox trigger value 0..255 to:
 *
 *   ((value << 8) | value) - 0x8000
 *
 * Preserve the original port's strict:
 *
 *   value > 0x20
 */
#define TRIGGER_AXIS_DEADZONE \
    ((BUTTON_DEADZONE * SDL_JOYSTICK_AXIS_MAX) / 0xFF)

enum RaphnetN64Layout {
    RAPHNET_N64_LAYOUT_V1,
};

struct RaphnetAdapterProfile {
    const char *name;
    enum RaphnetN64Layout layout;
    bool supports_raw_si;
    Uint8 raw_si_channel;
};

/* SDL normalizes each listed adapter to the same N64 joystick contract. New
 * models must be validated in the SDL backend before adding a profile here. */
static const struct RaphnetAdapterProfile sRaphnetProfiles[] = {
    { "Raphnet GC/N64 USB v3", RAPHNET_N64_LAYOUT_V1, true, 0 },
};

static SDL_GameController *sXboxController = NULL;
static SDL_Joystick *sRaphnetController = NULL;
static const struct RaphnetAdapterProfile *sRaphnetProfile = NULL;
static SDL_JoystickID sXboxControllerInstance = -1;
static bool sXboxControllerInitialized = false;
static bool sXboxControllerEverConnected = false;

#ifdef ENABLE_RUMBLE
#define XBOX_RUMBLE_ON_STRENGTH 0xFFFF
#define XBOX_RUMBLE_REFRESH_MS 100

static bool sRaphnetRumbleInitialized = false;
static bool sRaphnetRumbleEnabled = false;

static Uint8 raphnet_n64_accessory_crc(const Uint8 *data) {
    Uint8 crc = 0;

    for (int i = 0; i < 33; ++i) {
        const Uint8 value = i < 32 ? data[i] : 0;

        for (int bit = 0; bit < 8; ++bit) {
            const Uint8 carry = crc & 0x80;
            crc = (Uint8) ((crc << 1) | ((value >> (7 - bit)) & 1));

            if (carry) {
                crc ^= 0x85;
            }
        }
    }

    return crc;
}

static bool raphnet_n64_pak_write(
    Uint8 address_high,
    Uint8 address_low,
    Uint8 fill
) {
    Uint8 command[35];
    Uint8 response[1];
    Uint8 payload[32];

    command[0] = 0x03;
    command[1] = address_high;
    command[2] = address_low;
    memset(command + 3, fill, 32);
    memset(payload, fill, sizeof(payload));

    const int response_length = SDL_RaphnetN64RawSI(
        sRaphnetController,
        sRaphnetProfile->raw_si_channel,
        command,
        sizeof(command),
        response,
        sizeof(response)
    );

    return response_length == 1 &&
        response[0] == raphnet_n64_accessory_crc(payload);
}

static bool raphnet_n64_rumble_init(void) {
    Uint8 identify_command = 0x00;
    Uint8 identify_response[3];
    Uint8 signature_command[3] = { 0x02, 0x80, 0x01 };
    Uint8 signature_response[33];

    if (
        sRaphnetController == NULL ||
        sRaphnetProfile == NULL ||
        !sRaphnetProfile->supports_raw_si
    ) {
        return false;
    }

    int response_length = SDL_RaphnetN64RawSI(
        sRaphnetController,
        sRaphnetProfile->raw_si_channel,
        &identify_command,
        sizeof(identify_command),
        identify_response,
        sizeof(identify_response)
    );

    if (
        response_length != 3 ||
        identify_response[0] != 0x05 ||
        identify_response[1] != 0x00 ||
        (identify_response[2] & 0x01) == 0
    ) {
        return false;
    }

    response_length = SDL_RaphnetN64RawSI(
        sRaphnetController,
        sRaphnetProfile->raw_si_channel,
        signature_command,
        sizeof(signature_command),
        signature_response,
        sizeof(signature_response)
    );

    if (
        response_length != 33 ||
        signature_response[32] != raphnet_n64_accessory_crc(signature_response)
    ) {
        return false;
    }

    for (int i = 0; i < 32; ++i) {
        if (signature_response[i] != 0x80) {
            return false;
        }
    }

    if (!raphnet_n64_pak_write(0x80, 0x01, 0x80)) {
        return false;
    }

    sRaphnetRumbleInitialized = true;
    return true;
}

static void raphnet_n64_set_rumble(bool enabled) {
    if (sRaphnetController == NULL) {
        return;
    }

    if (enabled && !sRaphnetRumbleInitialized) {
        if (!raphnet_n64_rumble_init()) {
            return;
        }
    }

    if (!sRaphnetRumbleInitialized || enabled == sRaphnetRumbleEnabled) {
        return;
    }

    if (raphnet_n64_pak_write(0xC0, 0x1B, enabled ? 0x01 : 0x00)) {
        sRaphnetRumbleEnabled = enabled;
    }
}

void controller_xbox_set_rumble(
    u8 low_frequency_enabled,
    u8 high_frequency_enabled
) {
    if (!sXboxControllerInitialized) {
        return;
    }

    if (sRaphnetController != NULL) {
        /* A real N64 Rumble Pak is binary. Preserve the original on/off pulse
         * timing and issue the native accessory command without translating
         * it through Xbox low/high motor strengths or an SDL duration. */
        raphnet_n64_set_rumble(
            low_frequency_enabled || high_frequency_enabled
        );
        return;
    }

    if (sXboxController != NULL) {
        const Uint16 low_strength =
            low_frequency_enabled ? XBOX_RUMBLE_ON_STRENGTH : 0;

        const Uint16 high_strength =
            high_frequency_enabled ? XBOX_RUMBLE_ON_STRENGTH : 0;

        const Uint32 duration =
            (low_frequency_enabled || high_frequency_enabled)
                ? XBOX_RUMBLE_REFRESH_MS
                : 0;

        SDL_GameControllerRumble(
            sXboxController,
            low_strength,
            high_strength,
            duration
        );
    }
}

static void controller_xbox_shutdown(void) {
    controller_xbox_set_rumble(FALSE, FALSE);

    if (sRaphnetRumbleEnabled) {
        controller_xbox_set_rumble(FALSE, FALSE);
    }
}
#endif

static inline bool xbox_button_pressed(
    SDL_GameController *pad,
    SDL_GameControllerButton button
) {
    return SDL_GameControllerGetButton(
        pad,
        button
    ) != 0;
}

static inline bool xbox_trigger_pressed(
    SDL_GameController *pad,
    SDL_GameControllerAxis axis
) {
    return SDL_GameControllerGetAxis(
        pad,
        axis
    ) > TRIGGER_AXIS_DEADZONE;
}

static void controller_xbox_close_current(void) {
#ifdef ENABLE_RUMBLE
    controller_xbox_set_rumble(FALSE, FALSE);

    if (sRaphnetRumbleEnabled) {
        controller_xbox_set_rumble(FALSE, FALSE);
    }
#endif

    if (sXboxController != NULL) {
        SDL_GameControllerClose(
            sXboxController
        );

        sXboxController = NULL;
    }

    if (sRaphnetController != NULL) {
        SDL_JoystickClose(sRaphnetController);
        sRaphnetController = NULL;
        sRaphnetProfile = NULL;
#ifdef ENABLE_RUMBLE
        sRaphnetRumbleInitialized = false;
        sRaphnetRumbleEnabled = false;
#endif
    }

    sXboxControllerInstance = -1;
}

static bool controller_xbox_has_current(void) {
    return sXboxController != NULL || sRaphnetController != NULL;
}

static const struct RaphnetAdapterProfile *controller_xbox_raphnet_profile(
    int device_index
) {
    const char *name = SDL_JoystickNameForIndex(device_index);

    if (name == NULL) {
        return NULL;
    }

    for (
        size_t i = 0;
        i < sizeof(sRaphnetProfiles) / sizeof(sRaphnetProfiles[0]);
        ++i
    ) {
        if (strcmp(name, sRaphnetProfiles[i].name) == 0) {
            return &sRaphnetProfiles[i];
        }
    }

    return NULL;
}

static void controller_xbox_open_first(void) {
    if (controller_xbox_has_current()) {
        return;
    }

    const int count = SDL_NumJoysticks();

    for (int i = 0; i < count; ++i) {
        /* Gameplay belongs only to physical Xbox port 1. */
        if (SDL_JoystickGetDevicePlayerIndex(i) != 0) {
            continue;
        }

        /* Raphnet must use its normalized N64 joystick layout even if SDL
         * also has a GameController mapping for the adapter GUID. */
        const struct RaphnetAdapterProfile *raphnet_profile =
            controller_xbox_raphnet_profile(i);

        if (raphnet_profile != NULL) {
            SDL_Joystick *joystick = SDL_JoystickOpen(i);
            if (joystick == NULL) {
                continue;
            }

            sRaphnetController = joystick;
            sRaphnetProfile = raphnet_profile;
            sXboxControllerInstance = SDL_JoystickInstanceID(joystick);
            sXboxControllerEverConnected = true;
            return;
        }

        if (!SDL_IsGameController(i)) {
            continue;
        }

        SDL_GameController *controller =
            SDL_GameControllerOpen(i);

        if (controller == NULL) {
            continue;
        }

        SDL_Joystick *joystick =
            SDL_GameControllerGetJoystick(
                controller
            );

        if (joystick == NULL) {
            SDL_GameControllerClose(
                controller
            );

            continue;
        }

        sXboxController = controller;

        sXboxControllerInstance =
            SDL_JoystickInstanceID(
                joystick
            );

        sXboxControllerEverConnected = true;

        return;
    }
}

static void controller_xbox_update_device(void) {
    SDL_Event event;

    while (SDL_PollEvent(&event)) {
        if ((
            event.type ==
                SDL_CONTROLLERDEVICEREMOVED &&
            controller_xbox_has_current() &&
            event.cdevice.which ==
                sXboxControllerInstance
        ) || (
            event.type == SDL_JOYDEVICEREMOVED &&
            controller_xbox_has_current() &&
            event.jdevice.which == sXboxControllerInstance
        )) {
            controller_xbox_close_current();
        }
    }

    if (!controller_xbox_has_current()) {
        controller_xbox_open_first();
    }

    SDL_GameControllerUpdate();
}

static void controller_xbox_set_left_stick(
    OSContPad *pad,
    int16_t lx,
    int16_t ly
) {
    const float lx_float = (float)lx;
    const float ly_float = (float)ly;
    const float magnitude = sqrtf(lx_float * lx_float + ly_float * ly_float);

    if (magnitude > STICK_DEADZONE) {
        float scale = 1.0f;
        if (magnitude < STICK_FULL_SCALE) {
            const float scaled_magnitude =
                (magnitude - STICK_DEADZONE) * STICK_FULL_SCALE /
                (STICK_FULL_SCALE - STICK_DEADZONE);
            scale = scaled_magnitude / magnitude;
        }
        pad->stick_x = (s8)(lx_float * scale / 0x100);
        pad->stick_y = (s8)(ly_float * scale / 0x100);
    }
}

static bool raphnet_button_pressed(enum RaphnetN64SDLButton button) {
    return SDL_JoystickGetButton(sRaphnetController, button) != 0;
}

static void controller_xbox_read_raphnet(OSContPad *pad) {
    if (
        sRaphnetProfile == NULL ||
        sRaphnetProfile->layout != RAPHNET_N64_LAYOUT_V1
    ) {
        return;
    }

    if (raphnet_button_pressed(RAPHNET_N64_BUTTON_A)) pad->button |= A_BUTTON;
    if (raphnet_button_pressed(RAPHNET_N64_BUTTON_B)) pad->button |= B_BUTTON;
    if (raphnet_button_pressed(RAPHNET_N64_BUTTON_Z)) pad->button |= Z_TRIG;
    if (raphnet_button_pressed(RAPHNET_N64_BUTTON_START)) pad->button |= START_BUTTON;
    if (raphnet_button_pressed(RAPHNET_N64_BUTTON_L)) pad->button |= L_TRIG;
    if (raphnet_button_pressed(RAPHNET_N64_BUTTON_R)) pad->button |= R_TRIG;
    if (raphnet_button_pressed(RAPHNET_N64_BUTTON_C_UP)) pad->button |= U_CBUTTONS;
    if (raphnet_button_pressed(RAPHNET_N64_BUTTON_C_DOWN)) pad->button |= D_CBUTTONS;
    if (raphnet_button_pressed(RAPHNET_N64_BUTTON_C_LEFT)) pad->button |= L_CBUTTONS;
    if (raphnet_button_pressed(RAPHNET_N64_BUTTON_C_RIGHT)) pad->button |= R_CBUTTONS;
    if (raphnet_button_pressed(RAPHNET_N64_BUTTON_DPAD_UP)) pad->button |= U_JPAD;
    if (raphnet_button_pressed(RAPHNET_N64_BUTTON_DPAD_DOWN)) pad->button |= D_JPAD;
    if (raphnet_button_pressed(RAPHNET_N64_BUTTON_DPAD_LEFT)) pad->button |= L_JPAD;
    if (raphnet_button_pressed(RAPHNET_N64_BUTTON_DPAD_RIGHT)) pad->button |= R_JPAD;

    const int16_t lx = SDL_JoystickGetAxis(
        sRaphnetController,
        RAPHNET_N64_AXIS_STICK_X
    );
    const int16_t ly = (int16_t)~SDL_JoystickGetAxis(
        sRaphnetController,
        RAPHNET_N64_AXIS_STICK_Y
    );
    controller_xbox_set_left_stick(pad, lx, ly);
}

static void controller_xbox_init(void) {
    if (
        SDL_Init(
            SDL_INIT_GAMECONTROLLER
        ) != 0
    ) {
        return;
    }

    SDL_GameControllerEventState(
        SDL_ENABLE
    );

    sXboxControllerInitialized = true;

#ifdef ENABLE_RUMBLE
    atexit(controller_xbox_shutdown);
#endif

    controller_xbox_open_first();
}

static void controller_xbox_read(
    OSContPad *pad
) {
    if (!sXboxControllerInitialized) {
        return;
    }

    controller_xbox_update_device();

    SDL_GameController *xpad =
        sXboxController;

    if (!controller_xbox_has_current()) {
        if (sXboxControllerEverConnected) {
            pad->errnum = CONT_NO_RESPONSE_ERROR;
        }

        return;
    }

    if (sRaphnetController != NULL) {
        controller_xbox_read_raphnet(pad);
        return;
    }

    const bool xpad_black =
        xbox_button_pressed(
            xpad,
            SDL_CONTROLLER_BUTTON_RIGHTSHOULDER
        );

    const bool xpad_ltrig =
        xbox_trigger_pressed(
            xpad,
            SDL_CONTROLLER_AXIS_TRIGGERLEFT
        );

    const bool xpad_rtrig =
        xbox_trigger_pressed(
            xpad,
            SDL_CONTROLLER_AXIS_TRIGGERRIGHT
        );

    /*
     * Preserve the original dashboard reboot:
     *
     * Back + Black + left trigger + right trigger
     */
    if (
        xbox_button_pressed(
            xpad,
            SDL_CONTROLLER_BUTTON_BACK
        ) &&
        xpad_black &&
        xpad_ltrig &&
        xpad_rtrig
    ) {
#ifdef ENABLE_RUMBLE
        controller_xbox_set_rumble(FALSE, FALSE);
#endif
        XReboot();
    }

    /*
     * Preserve Baseline-B mappings.
     */

    if (
        xbox_button_pressed(
            xpad,
            SDL_CONTROLLER_BUTTON_A
        )
    ) {
        pad->button |= A_BUTTON;
    }

    if (
        xbox_button_pressed(
            xpad,
            SDL_CONTROLLER_BUTTON_X
        )
    ) {
        pad->button |= B_BUTTON;
    }

    if (
        xbox_button_pressed(
            xpad,
            SDL_CONTROLLER_BUTTON_LEFTSHOULDER
        )
    ) {
        pad->button |= L_TRIG;
    }

    if (
        xpad_black ||
        xpad_rtrig
    ) {
        pad->button |= R_TRIG;
    }

    if (xpad_ltrig) {
        pad->button |= Z_TRIG;
    }

    if (
        xbox_button_pressed(
            xpad,
            SDL_CONTROLLER_BUTTON_START
        )
    ) {
        pad->button |= START_BUTTON;
    }

    /*
     * nxdk SDL changes Xbox Y from its original signed
     * orientation to SDL orientation using bitwise NOT.
     *
     * Undo that transformation so the existing SM64 Xbox
     * behavior sees the same values as Baseline B.
     */

    const int16_t lx =
        SDL_GameControllerGetAxis(
            xpad,
            SDL_CONTROLLER_AXIS_LEFTX
        );

    const int16_t ly =
        (int16_t)~SDL_GameControllerGetAxis(
            xpad,
            SDL_CONTROLLER_AXIS_LEFTY
        );

    const int16_t rx =
        SDL_GameControllerGetAxis(
            xpad,
            SDL_CONTROLLER_AXIS_RIGHTX
        );

    const int16_t ry =
        (int16_t)~SDL_GameControllerGetAxis(
            xpad,
            SDL_CONTROLLER_AXIS_RIGHTY
        );

    /*
     * Treat the right stick like the N64's four discrete C buttons.
     *
     * Keep the existing 0x4000 cardinal activation distance, but use
     * a circular center dead zone so the four directional regions are
     * larger around the diagonals than the old per-axis square gate.
     *
     * Once outside the dead zone, only the dominant axis contributes
     * a C button. This prevents an ordinary diagonal stick position
     * from pressing two adjacent C buttons at once.
     */
    const uint32_t rx_abs =
        (rx < 0) ? (uint32_t)(-(int32_t)rx) : (uint32_t)rx;

    const uint32_t ry_abs =
        (ry < 0) ? (uint32_t)(-(int32_t)ry) : (uint32_t)ry;

    const uint32_t c_stick_magnitude_sq =
        rx_abs * rx_abs +
        ry_abs * ry_abs;

    if (
        c_stick_magnitude_sq >
            (uint32_t)C_STICK_DEADZONE *
            (uint32_t)C_STICK_DEADZONE
    ) {
        if (rx_abs >= ry_abs) {
            if (rx < 0) {
                pad->button |= L_CBUTTONS;
            } else {
                pad->button |= R_CBUTTONS;
            }
        } else {
            if (ry < 0) {
                pad->button |= D_CBUTTONS;
            } else {
                pad->button |= U_CBUTTONS;
            }
        }
    }

    controller_xbox_set_left_stick(pad, lx, ly);
}

struct ControllerAPI controller_xbox = {
    controller_xbox_init,
    controller_xbox_read
};

#endif
