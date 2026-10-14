#if defined(TARGET_XBOX)

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include <hal/xbox.h>
#include <SDL.h>

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
#define TRIGGER_AXIS_DEADZONE ((BUTTON_DEADZONE * SDL_JOYSTICK_AXIS_MAX) / 0xFF)

enum RaphnetN64Layout {
    RAPHNET_N64_LAYOUT_V1,
};

struct RaphnetAdapterProfile {
    const char *name;
    enum RaphnetN64Layout layout;
};

/* SDL normalizes each listed adapter to the same N64 joystick contract. New
 * models must be validated in the SDL backend before adding a profile here. */
static const struct RaphnetAdapterProfile sRaphnetProfiles[] = {
    { "Raphnet GC/N64 USB v3", RAPHNET_N64_LAYOUT_V1 },
};

static SDL_GameController *sXboxController = NULL;
static SDL_Joystick *sRaphnetController = NULL;
static const struct RaphnetAdapterProfile *sRaphnetProfile = NULL;
static SDL_JoystickID sXboxControllerInstance = -1;
static bool sXboxControllerInitialized = false;
static bool sXboxControllerEverConnected = false;

static inline bool xbox_button_pressed(SDL_GameController *pad, SDL_GameControllerButton button) {
    return SDL_GameControllerGetButton(pad, button) != 0;
}

static inline bool xbox_trigger_pressed(SDL_GameController *pad, SDL_GameControllerAxis axis) {
    return SDL_GameControllerGetAxis(pad, axis) > TRIGGER_AXIS_DEADZONE;
}

static void controller_xbox_close_current(void) {
    if (sXboxController != NULL) {
        SDL_GameControllerClose(sXboxController);

        sXboxController = NULL;
    }

    if (sRaphnetController != NULL) {
        SDL_JoystickClose(sRaphnetController);
        sRaphnetController = NULL;
        sRaphnetProfile = NULL;
    }

    sXboxControllerInstance = -1;
}

static bool controller_xbox_has_current(void) {
    return sXboxController != NULL || sRaphnetController != NULL;
}

static const struct RaphnetAdapterProfile *controller_xbox_raphnet_profile(int device_index) {
    const char *name = SDL_JoystickNameForIndex(device_index);

    if (name == NULL) {
        return NULL;
    }

    for (size_t i = 0; i < sizeof(sRaphnetProfiles) / sizeof(sRaphnetProfiles[0]); ++i) {
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
        const struct RaphnetAdapterProfile *raphnet_profile = controller_xbox_raphnet_profile(i);

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

        SDL_GameController *controller = SDL_GameControllerOpen(i);

        if (controller == NULL) {
            continue;
        }

        SDL_Joystick *joystick = SDL_GameControllerGetJoystick(controller);

        if (joystick == NULL) {
            SDL_GameControllerClose(controller);

            continue;
        }

        sXboxController = controller;

        sXboxControllerInstance = SDL_JoystickInstanceID(joystick);

        sXboxControllerEverConnected = true;

        return;
    }
}

static void controller_xbox_update_device(void) {
    SDL_Event event;

    while (SDL_PollEvent(&event)) {
        if ((event.type == SDL_CONTROLLERDEVICEREMOVED && controller_xbox_has_current()
             && event.cdevice.which == sXboxControllerInstance)
            || (event.type == SDL_JOYDEVICEREMOVED && controller_xbox_has_current()
                && event.jdevice.which == sXboxControllerInstance)) {
            controller_xbox_close_current();
        }
    }

    if (!controller_xbox_has_current()) {
        controller_xbox_open_first();
    }

    SDL_GameControllerUpdate();
}

static void controller_xbox_set_left_stick(OSContPad *pad, int16_t lx, int16_t ly) {
    const float lx_float = (float) lx;
    const float ly_float = (float) ly;
    const float magnitude = sqrtf(lx_float * lx_float + ly_float * ly_float);

    if (magnitude > STICK_DEADZONE) {
        float scale = 1.0f;
        if (magnitude < STICK_FULL_SCALE) {
            const float scaled_magnitude =
                (magnitude - STICK_DEADZONE) * STICK_FULL_SCALE / (STICK_FULL_SCALE - STICK_DEADZONE);
            scale = scaled_magnitude / magnitude;
        }
        pad->stick_x = (s8) (lx_float * scale / 0x100);
        pad->stick_y = (s8) (ly_float * scale / 0x100);
    }
}

static bool raphnet_button_pressed(enum RaphnetN64SDLButton button) {
    return SDL_JoystickGetButton(sRaphnetController, button) != 0;
}

static void controller_xbox_read_raphnet(OSContPad *pad) {
    if (sRaphnetProfile == NULL || sRaphnetProfile->layout != RAPHNET_N64_LAYOUT_V1) {
        return;
    }

    if (raphnet_button_pressed(RAPHNET_N64_BUTTON_A))
        pad->button |= A_BUTTON;
    if (raphnet_button_pressed(RAPHNET_N64_BUTTON_B))
        pad->button |= B_BUTTON;
    if (raphnet_button_pressed(RAPHNET_N64_BUTTON_Z))
        pad->button |= Z_TRIG;
    if (raphnet_button_pressed(RAPHNET_N64_BUTTON_START))
        pad->button |= START_BUTTON;
    if (raphnet_button_pressed(RAPHNET_N64_BUTTON_L))
        pad->button |= L_TRIG;
    if (raphnet_button_pressed(RAPHNET_N64_BUTTON_R))
        pad->button |= R_TRIG;
    if (raphnet_button_pressed(RAPHNET_N64_BUTTON_C_UP))
        pad->button |= U_CBUTTONS;
    if (raphnet_button_pressed(RAPHNET_N64_BUTTON_C_DOWN))
        pad->button |= D_CBUTTONS;
    if (raphnet_button_pressed(RAPHNET_N64_BUTTON_C_LEFT))
        pad->button |= L_CBUTTONS;
    if (raphnet_button_pressed(RAPHNET_N64_BUTTON_C_RIGHT))
        pad->button |= R_CBUTTONS;
    if (raphnet_button_pressed(RAPHNET_N64_BUTTON_DPAD_UP))
        pad->button |= U_JPAD;
    if (raphnet_button_pressed(RAPHNET_N64_BUTTON_DPAD_DOWN))
        pad->button |= D_JPAD;
    if (raphnet_button_pressed(RAPHNET_N64_BUTTON_DPAD_LEFT))
        pad->button |= L_JPAD;
    if (raphnet_button_pressed(RAPHNET_N64_BUTTON_DPAD_RIGHT))
        pad->button |= R_JPAD;

    const int16_t lx = SDL_JoystickGetAxis(sRaphnetController, RAPHNET_N64_AXIS_STICK_X);
    const int16_t ly = (int16_t) ~SDL_JoystickGetAxis(sRaphnetController, RAPHNET_N64_AXIS_STICK_Y);
    controller_xbox_set_left_stick(pad, lx, ly);
}

static void controller_xbox_init(void) {
    if (SDL_Init(SDL_INIT_GAMECONTROLLER) != 0) {
        return;
    }

    SDL_GameControllerEventState(SDL_ENABLE);

    sXboxControllerInitialized = true;

    controller_xbox_open_first();
}

static void controller_xbox_read(OSContPad *pad) {
    if (!sXboxControllerInitialized) {
        return;
    }

    controller_xbox_update_device();

    SDL_GameController *xpad = sXboxController;

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

    const bool xpad_black = xbox_button_pressed(xpad, SDL_CONTROLLER_BUTTON_RIGHTSHOULDER);

    const bool xpad_ltrig = xbox_trigger_pressed(xpad, SDL_CONTROLLER_AXIS_TRIGGERLEFT);

    const bool xpad_rtrig = xbox_trigger_pressed(xpad, SDL_CONTROLLER_AXIS_TRIGGERRIGHT);

    /*
     * Preserve the original dashboard reboot:
     *
     * Back + Black + left trigger + right trigger
     */
    if (xbox_button_pressed(xpad, SDL_CONTROLLER_BUTTON_BACK) && xpad_black && xpad_ltrig
        && xpad_rtrig) {
        XReboot();
    }

    /*
     * Preserve Baseline-B mappings.
     */

    if (xbox_button_pressed(xpad, SDL_CONTROLLER_BUTTON_A)) {
        pad->button |= A_BUTTON;
    }

    if (xbox_button_pressed(xpad, SDL_CONTROLLER_BUTTON_X)) {
        pad->button |= B_BUTTON;
    }

    if (xbox_button_pressed(xpad, SDL_CONTROLLER_BUTTON_LEFTSHOULDER)) {
        pad->button |= L_TRIG;
    }

    if (xpad_black || xpad_rtrig) {
        pad->button |= R_TRIG;
    }

    if (xpad_ltrig) {
        pad->button |= Z_TRIG;
    }

    if (xbox_button_pressed(xpad, SDL_CONTROLLER_BUTTON_START)) {
        pad->button |= START_BUTTON;
    }

    /*
     * nxdk SDL changes Xbox Y from its original signed
     * orientation to SDL orientation using bitwise NOT.
     *
     * Undo that transformation so the existing SM64 Xbox
     * behavior sees the same values as Baseline B.
     */

    const int16_t lx = SDL_GameControllerGetAxis(xpad, SDL_CONTROLLER_AXIS_LEFTX);

    const int16_t ly = (int16_t) ~SDL_GameControllerGetAxis(xpad, SDL_CONTROLLER_AXIS_LEFTY);

    const int16_t rx = SDL_GameControllerGetAxis(xpad, SDL_CONTROLLER_AXIS_RIGHTX);

    const int16_t ry = (int16_t) ~SDL_GameControllerGetAxis(xpad, SDL_CONTROLLER_AXIS_RIGHTY);

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
    const uint32_t rx_abs = (rx < 0) ? (uint32_t) (-(int32_t) rx) : (uint32_t) rx;

    const uint32_t ry_abs = (ry < 0) ? (uint32_t) (-(int32_t) ry) : (uint32_t) ry;

    const uint32_t c_stick_magnitude_sq = rx_abs * rx_abs + ry_abs * ry_abs;

    if (c_stick_magnitude_sq > (uint32_t) C_STICK_DEADZONE * (uint32_t) C_STICK_DEADZONE) {
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
