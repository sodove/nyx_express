; VESC Express / RadioStack ESP32-C3 Super Mini
; WS2812B data: GPIO10 (use DIN on the first pixel)
;
; The strip must have its own 5 V supply. Tie its GND to the ESP32 GND.
; For a 5 V strip, a 3.3 V -> 5 V AHCT/HCT level shifter is recommended.

(def ws-pin 10)
(def led-count 8)

; type 0 = GRB (WS2812/WS2812B), timing 1 = WS2812B, gamma = 1
(rgbled-init ws-pin 1)
(def leds (rgbled-buffer led-count 0 1))

(defun fill-strip (color) {
    (looprange i 0 led-count {
        (rgbled-color leds i color)
    })
    (rgbled-update leds ws-pin)
})

; Simple proof-of-life animation.
(loopwhile t {
    (fill-strip (color-make 1.0 0.0 0.0))
    (sleep 1.0)
    (fill-strip (color-make 0.0 1.0 0.0))
    (sleep 1.0)
    (fill-strip (color-make 0.0 0.0 1.0))
    (sleep 1.0)
})
