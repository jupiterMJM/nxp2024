"""
fichier pas encore testé; à tester très rapidement
bien mettre toutes les LED en série (pour la donnée) et en parallèle pour l'alim
ne pas oublier de relier la masse de la rpi à la masse des LED!!!!!
"""

import board
import neopixel
import random
import time

# LED strip configuration
PIN = board.D2  # GPIO pin connected to the NeoPixels (physical pin 2 corresponds to GPIO2 on Pi)
NUMPIXELS = 12  # Number of NeoPixels
DELAYVAL = 0.1  # Delay in seconds (100 milliseconds)

# Create the NeoPixel object
pixels = neopixel.NeoPixel(PIN, NUMPIXELS, auto_write=False)

# Function to set random color
def set_color():
    red = random.randint(0, 255)
    green = random.randint(0, 255)
    blue = random.randint(0, 255)
    return (red, green, blue)

# Main loop
while True:
    color = set_color()  # Generate random color

    # Loop over each pixel in the strip
    for i in range(NUMPIXELS):
        pixels[i] = color  # Set pixel to the generated color
        pixels.show()  # Push the color to the LEDs
        time.sleep(DELAYVAL)  # Wait for a short period
