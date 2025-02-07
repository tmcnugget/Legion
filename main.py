# -----|>MAIN IMPORTS<|-----
import time

# -----|>JOYSTICK SCRIPT<|-----
import pygame

os.environ["SDL_VIDEODRIVER"] = "dummy"

pygame.init()
pygame.joystick.init()

joysticks = {}

def deadzone(number):
    if abs(number) < 0.005:
        return 0
    return number

def joystick_loop:
    for event in pygame.event.get():
        if event.type == pygame.JOYDEVICEADDED:
            # A new joystick has been connected
            joy = pygame.joystick.Joystick(event.device_index)
            joysticks[joy.get_instance_id()] = joy
            print(f"Joystick {joy.get_instance_id()} connected")

        if event.type == pygame.JOYDEVICEREMOVED:
            # A joystick has been disconnected
            del joysticks[event.instance_id]
            print(f"Joystick {event.instance_id} disconnected")

    for joystick in joysticks.values():
        x1 = deadzone(round(joystick.get_axis(0), 3)) / 2 * speed # Left/Right
        y = deadzone(round(joystick.get_axis(1), 3)) / 2 * speed # Up/Down
        x2 = -deadzone(round(joystick.get_axis(2), 3)) / 2 * speed # Rotate

        zl = joystick.get_button(6)
        zr = joystick.get_button(7)

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

# -----|>STARTUP SCRIPT<|-----
def setup():
    buzzer_startup()

# -----|>LOOP SCRIPT<|-----
def loop():
    rainbow_cycle(0.005)
    joystick_loop()

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
