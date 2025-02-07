# -----|>MAIN IMPORTS<|-----
import time

# -----|>LED SCRIPT<|-----
import board
import neopixel

# LED Configuration
LED_PIN = board.D18      # Use the board's pin definition
NUM_LEDS = 7             # Number of LEDs
ORDER = neopixel.GRB     # Color order (WS2812/WS2811 uses GRB)

# Initialize the LED strip
pixels = neopixel.NeoPixel(LED_PIN, NUM_LEDS, brightness=0.5, auto_write=False, pixel_order=ORDER)

def rainbow_cycle(wait):
    """Displays a rainbow effect"""
    for j in range(255):
        for i in range(NUM_LEDS):
            pixel_index = (i * 256 // NUM_LEDS) + j
            pixels[i] = wheel(pixel_index & 255)
        pixels.show()
        time.sleep(wait)

def wheel(pos):
    """Color wheel to generate rainbow colors"""
    if pos < 85:
        return (pos * 3, 255 - pos * 3, 0)
    elif pos < 170:
        pos -= 85
        return (255 - pos * 3, 0, pos * 3)
    else:
        pos -= 170
        return (0, pos * 3, 255 - pos * 3)

# -----|>BUZZER SCRIPT<|-----
from gpiozero import Buzzer

buzzer = Buzzer(17)

def buzzer_startup():
    buzzer.on()
    time.sleep(0.5)
    buzzer.off()
    time.sleep(1)
    buzzer.on()
    time.sleep(0.2)
    buzzer.off
    time.sleep(0.2)
    buzzer.on()
    time.sleep(0.2)
    buzzer.off
    time.sleep(0.2)
    buzzer.on()
    time.sleep(0.2)
    buzzer.off

# -----|>STARTUP SCRIPT<|-----
def setup():
    buzzer_startup()

# -----|>LOOP SCRIPT<|-----
def loop():
    rainbow_cycle(0.005)

# -----|>SHUTDOWN SCRIPT<|-----
def cleanup():
    pixels.fill((0, 0, 0))
    pixels.show()

# -----|>EVENT TRIGGERS<|-----
try:
    setup()
    while True:
        loop()
except KeyboardInterrupt:
    cleanup()
