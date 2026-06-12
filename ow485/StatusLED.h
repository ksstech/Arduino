/**
 * StatusLED.h  —  Multi-channel non-blocking LED sequencer for ATtiny3224
 *                 (megaTinyCore 2.6.x)
 *
 * DESIGN
 * ──────
 * All per-channel state is consolidated in LEDChannel.  The channel count
 * is a compile-time template parameter; the compiler generates exactly the
 * code needed for N channels with no dynamic allocation.
 *
 * Implementation lives entirely in this header because C++ requires template
 * definitions to be visible at the point of instantiation.  On a small MCU
 * this is preferable to explicit instantiation boilerplate.
 *
 * USAGE
 * ─────
 *   static const uint8_t kLEDPins[] = { PIN_PA5, PIN_PA3 };
 *   StatusLED<2> leds(kLEDPins);
 *
 *   leds.configure(0, LED_INFINITE, 1, 2, 1, 0);  // channel 0: slow breath
 *   leds.configure(1, 3, 0, 1, 0, 1);             // channel 1: 3 fast blinks
 *
 *   // In loop():
 *   leds.update();
 *
 * PWM NOTE
 * ────────
 * Hardware PWM is driven via analogWrite() which uses TCA0 split mode.
 * On the 14-pin ATtiny3224, PA5 = WO5 and PA3 = WO3 (alternate routing).
 * millis() MUST be assigned to TCB0 in the Arduino IDE
 * (Tools → millis()/micros() → TCB0) so TCA0 is fully available for PWM.
 */

#pragma once
#include <Arduino.h>

/* ── Sentinel values for the 'repeats' parameter ────────────────────────── */
static constexpr uint16_t LED_STOP     = 0x0000u;  ///< Stop and turn off immediately
static constexpr uint16_t LED_INFINITE = 0xFFFFu;  ///< Run indefinitely

/* ── Sequencer stage identifiers ────────────────────────────────────────── */
enum class LEDStage : uint8_t {
    IDLE     = 0,
    FADE_IN  = 1,
    FULL_ON  = 2,
    FADE_OUT = 3,
    FULL_OFF = 4,
};

/* ── Per-channel state (ALL state in one place) ─────────────────────────── */
/**
 * LEDChannel consolidates every variable that belongs to one LED output.
 * This struct is the single source of truth for both configuration and
 * runtime state; there are no loose member variables in StatusLED.
 *
 * Fields marked [config] are written by configure() and held constant
 * while the sequence runs.  Fields marked [runtime] are modified by the
 * state machine during update().
 */
struct LEDChannel {
    /* [config] Hardware */
    uint8_t  pin;           ///< Arduino pin number (must support analogWrite)

    /* [config] Sequence parameters */
    uint16_t repeats;       ///< 0=stop, 65535=infinite, else exact count
    uint32_t fadeInMs;      ///< Fade-in  duration (milliseconds)
    uint32_t onMs;          ///< Full-on  duration (milliseconds)
    uint32_t fadeOutMs;     ///< Fade-out duration (milliseconds)
    uint32_t offMs;         ///< Full-off duration (milliseconds)

    /* [runtime] State machine */
    LEDStage stage;         ///< Current stage
    uint32_t stageStartMs;  ///< millis() value when current stage began
    uint16_t cyclesDone;    ///< Complete cycles executed so far
    bool     running;       ///< False when IDLE or stopped
};

/* ── StatusLED template class ───────────────────────────────────────────── */
template<uint8_t N_CH>
class StatusLED {
    static_assert(N_CH > 0, "StatusLED: N_CH must be at least 1");

public:
    /**
     * Construct and initialise all channels.
     * @param pins  C-array of Arduino pin numbers, length exactly N_CH.
     *              Each pin must be capable of hardware PWM (analogWrite).
     *
     * Example:
     *   static const uint8_t pins[] = { PIN_PA5 };
     *   StatusLED<1> led(pins);
     */
    explicit StatusLED(const uint8_t (&pins)[N_CH])
    {
        for (uint8_t i = 0; i < N_CH; ++i) {
            _ch[i]         = {};          /* zero all fields */
            _ch[i].pin     = pins[i];
            _ch[i].stage   = LEDStage::IDLE;
            pinMode(pins[i], OUTPUT);
            analogWrite(pins[i], 0);
        }
    }

    /* ── Configuration & control ─────────────────────────────────────── */

    /**
     * Configure and immediately start (or stop) one channel.
     *
     * @param ch         Channel index (0 … N_CH-1).
     * @param repeats    LED_STOP (0) = turn off.
     *                   LED_INFINITE (65535) = run forever.
     *                   1–65534 = exact cycle count.
     * @param fadeInSec  Fade-in  duration in whole seconds (0 = skip).
     * @param onSec      Full-on  duration in whole seconds (0 = skip).
     * @param fadeOutSec Fade-out duration in whole seconds (0 = skip).
     * @param offSec     Full-off duration in whole seconds (0 = skip).
     */
    void configure(uint8_t  ch,
                   uint16_t repeats,
                   uint16_t fadeInSec,
                   uint16_t onSec,
                   uint16_t fadeOutSec,
                   uint16_t offSec)
    {
        if (ch >= N_CH) return;
        _ch[ch].repeats   = repeats;
        _ch[ch].fadeInMs  = (uint32_t)fadeInSec  * 1000UL;
        _ch[ch].onMs      = (uint32_t)onSec      * 1000UL;
        _ch[ch].fadeOutMs = (uint32_t)fadeOutSec * 1000UL;
        _ch[ch].offMs     = (uint32_t)offSec     * 1000UL;
        (repeats == LED_STOP) ? stop(ch) : start(ch);
    }

    /** Restart a channel from the beginning using its current configuration. */
    void start(uint8_t ch)
    {
        if (ch >= N_CH || _ch[ch].repeats == LED_STOP) return;
        _ch[ch].cyclesDone = 0;
        _ch[ch].running    = true;

        /* Enter the first stage that has a non-zero duration, so a config
           like (in=0, on=10s, ...) goes straight to FULL_ON with no
           momentary drive to 0 at startup.  The cycle order matches the
           enum order (FADE_IN=1 … FULL_OFF=4), which the rest of the class
           already relies on, so we iterate the enum values directly. */
        for (uint8_t s = (uint8_t)LEDStage::FADE_IN;
             s <= (uint8_t)LEDStage::FULL_OFF; ++s) {
            if (_stageDuration(ch, (LEDStage)s) > 0) {
                _enterStage(ch, (LEDStage)s);
                return;
            }
        }
        /* All stages zero — nothing to run */
        stop(ch);
    }

    /** Stop one channel immediately and turn its LED off. */
    void stop(uint8_t ch)
    {
        if (ch >= N_CH) return;
        _ch[ch].running = false;
        _ch[ch].stage   = LEDStage::IDLE;
        analogWrite(_ch[ch].pin, 0);
    }

    /** Stop all channels. */
    void stopAll()
    {
        for (uint8_t i = 0; i < N_CH; ++i) stop(i);
    }

    /**
     * Advance all channel state machines.
     * Call this every iteration of loop(); the more frequently it is called
     * the smoother the fades.  A minimum call rate of ~20 Hz is recommended.
     */
    void update()
    {
        for (uint8_t i = 0; i < N_CH; ++i) _updateChannel(i);
    }

    /* ── Accessors ───────────────────────────────────────────────────── */

    bool     isRunning(uint8_t ch)    const { return ch < N_CH && _ch[ch].running; }
    bool     isIdle(uint8_t ch)       const { return ch < N_CH && _ch[ch].stage == LEDStage::IDLE; }
    LEDStage getStage(uint8_t ch)     const { return ch < N_CH ? _ch[ch].stage : LEDStage::IDLE; }
    uint16_t getCycleCount(uint8_t ch) const { return ch < N_CH ? _ch[ch].cyclesDone : 0u; }

    /** Compile-time channel count; useful in range-checked loops. */
    static constexpr uint8_t channelCount() { return N_CH; }

    /** Direct read access to the underlying channel state for diagnostics. */
    const LEDChannel& channel(uint8_t ch) const { return _ch[ch < N_CH ? ch : 0]; }

private:
    LEDChannel _ch[N_CH];

    /* ── State machine internals ─────────────────────────────────────── */

    void _updateChannel(uint8_t ch)
    {
        if (!_ch[ch].running) return;

        /*
         * Drive the active (non-zero) stage.  When its time expires, advance
         * — and keep advancing through any zero-duration stages in the SAME
         * call, so a full cycle never costs extra loop() iterations and the
         * pin is never driven to an intermediate boundary value for a stage
         * that has no duration.  See _advance() for the skip logic.
         */
        const uint32_t elapsed = millis() - _ch[ch].stageStartMs;

        switch (_ch[ch].stage) {
            case LEDStage::FADE_IN:
                if (elapsed >= _ch[ch].fadeInMs) _advance(ch);
                else analogWrite(_ch[ch].pin, _lerp(elapsed, _ch[ch].fadeInMs, true));
                break;

            case LEDStage::FULL_ON:
                if (elapsed >= _ch[ch].onMs) _advance(ch);
                break;

            case LEDStage::FADE_OUT:
                if (elapsed >= _ch[ch].fadeOutMs) _advance(ch);
                else analogWrite(_ch[ch].pin, _lerp(elapsed, _ch[ch].fadeOutMs, false));
                break;

            case LEDStage::FULL_OFF:
                if (elapsed >= _ch[ch].offMs) _advance(ch);
                break;

            case LEDStage::IDLE:
            default:
                break;
        }
    }

    /** Duration (ms) for a given stage. */
    uint32_t _stageDuration(uint8_t ch, LEDStage s) const
    {
        switch (s) {
            case LEDStage::FADE_IN:  return _ch[ch].fadeInMs;
            case LEDStage::FULL_ON:  return _ch[ch].onMs;
            case LEDStage::FADE_OUT: return _ch[ch].fadeOutMs;
            case LEDStage::FULL_OFF: return _ch[ch].offMs;
            default:                 return 0;
        }
    }

    void _enterStage(uint8_t ch, LEDStage s)
    {
        _ch[ch].stage        = s;
        _ch[ch].stageStartMs = millis();

        /*
         * Only real (non-zero) stages reach _enterStage — _advance() skips
         * zero-duration stages without entering them — so every analogWrite
         * here belongs to a stage that will actually run.  This eliminates
         * the glitch-to-zero that previously occurred between repeats.
         */
        switch (s) {
            case LEDStage::FADE_IN:   analogWrite(_ch[ch].pin, 0);   break;
            case LEDStage::FULL_ON:   analogWrite(_ch[ch].pin, 255); break;
            case LEDStage::FADE_OUT:  analogWrite(_ch[ch].pin, 255); break;
            case LEDStage::FULL_OFF:  analogWrite(_ch[ch].pin, 0);   break;
            case LEDStage::IDLE:      analogWrite(_ch[ch].pin, 0);   break;
        }
    }

    /** Return the stage that follows 's' in the cycle (no repeat accounting). */
    static LEDStage _nextStage(LEDStage s)
    {
        switch (s) {
            case LEDStage::FADE_IN:  return LEDStage::FULL_ON;
            case LEDStage::FULL_ON:  return LEDStage::FADE_OUT;
            case LEDStage::FADE_OUT: return LEDStage::FULL_OFF;
            case LEDStage::FULL_OFF: return LEDStage::FADE_IN;
            default:                 return LEDStage::IDLE;
        }
    }

    void _advance(uint8_t ch)
    {
        /*
         * Walk forward through stages, accounting for cycle completion at the
         * FULL_OFF → FADE_IN boundary, and skipping any stage whose duration
         * is zero WITHOUT entering it or touching the pin.  Stops at the first
         * stage with a non-zero duration (which it enters) or when the run
         * finishes.
         *
         * Bounded by a guard counter so that an all-zero configuration cannot
         * spin forever.
         */
        LEDStage s = _ch[ch].stage;

        for (uint8_t guard = 0; guard < 8u; ++guard) {

            /* Completing FULL_OFF closes one cycle */
            if (s == LEDStage::FULL_OFF) {
                ++_ch[ch].cyclesDone;
                const bool infinite = (_ch[ch].repeats == LED_INFINITE);
                if (!infinite && _ch[ch].cyclesDone >= _ch[ch].repeats) {
                    stop(ch);
                    return;
                }
            }

            s = _nextStage(s);

            /* Enter the first stage that has real duration */
            if (_stageDuration(ch, s) > 0) {
                _enterStage(ch, s);
                return;
            }
            /* else: zero-duration stage — skip it, keep walking */
        }

        /*
         * All four stages are zero-duration: nothing to time.  Stop to avoid
         * a busy spin.  A caller wanting continuous output must give at least
         * one stage a non-zero duration.
         */
        stop(ch);
    }

    /**
     * Linear interpolation over [0, 255].
     * @param rising  true  → 0…255 (fade in)
     *                false → 255…0 (fade out)
     */
    static uint8_t _lerp(uint32_t elapsed, uint32_t duration, bool rising)
    {
        /* duration > 0 is guaranteed by the caller */
        uint32_t v = (elapsed * 255UL) / duration;
        if (v > 255u) v = 255u;
        const uint8_t u = static_cast<uint8_t>(v);
        return rising ? u : static_cast<uint8_t>(255u - u);
    }
};

/* ══════════════════════════════════════════════════════════════════════════
 * CHANGELOG
 * ══════════════════════════════════════════════════════════════════════════
 *
 * 2026-05-29
 *   - start(): replaced the {FADE_IN..FULL_OFF} lookup array with a direct
 *     enum-value iteration (FADE_IN=1 … FULL_OFF=4).  Removes a redundant
 *     array whose contents were always index+1; saves 4 bytes stack + an
 *     indirection per iteration.
 *
 * 2026-05-29 (earlier same day)
 *   - State machine reworked to skip zero-duration stages atomically:
 *       * _advance() now walks forward through stages in a single bounded
 *         loop, skipping any stage with duration 0 WITHOUT entering it or
 *         calling analogWrite.  Fixes (a) the momentary drive-to-0 glitch
 *         between repeats for non-LED loads (e.g. DC motors), and (b) the
 *         multiple loop() iterations previously needed to traverse zero
 *         stages — a full cycle now resolves in one update() call.
 *       * start() enters the first NON-ZERO stage directly (no startup
 *         glitch for configs like in=0, on=10s).
 *       * All-zero configurations call stop() instead of spinning; guarded
 *         by a counter so _advance() can never loop forever.
 *
 * Earlier, undated (development sequence, exact times not recorded):
 *   - Converted to a template class StatusLED<N_CH> with all per-channel
 *     state consolidated in the LEDChannel struct; channel count fixed at
 *     compile time, no dynamic allocation.  Implementation moved fully into
 *     the header (required for templates).
 *   - configure() API: (channel, repeats, fadeInSec, onSec, fadeOutSec,
 *     offSec).  repeats: 0=stop, 65535=infinite, else exact count.  Stage
 *     durations are uint16_t seconds, stored internally as uint32_t ms for
 *     millis() arithmetic.
 *   - Original single-channel version: hardware PWM via analogWrite (TCA0
 *     split mode, WO5→PA5), loop()-driven (no ISR), 5-stage sequence
 *     FADE_IN → FULL_ON → FADE_OUT → FULL_OFF → repeat/idle.
 */