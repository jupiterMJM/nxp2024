#!/usr/bin/env python3
#-- coding: utf-8 --

"""
code issu de https://raspberry-pi.fr/servomoteur-raspberry-pi/
confirmé par ChatGPT
ne fonctionne pas très bien bizarrement (selon Baptiste)
"""

import RPi.GPIO as GPIO
import time


#Set function to calculate percent from angle
def angle_to_percent (angle) :
	"""
	How the Servo Angle is Controlled:
    The PWM signal determines the angle of the servo:
    0°: 1 ms pulse width (approx. 2% duty cycle)
    90°: 1.5 ms pulse width (approx. 7.5% duty cycle)
    180°: 2 ms pulse width (approx. 12% duty cycle)
    The formula duty_cycle = 2 + (angle / 180) converts an angle to the corresponding duty cycle for typical servos.
	source: ChatGPT
	"""
	if angle > 180 or angle < 0 :
		return False

	start = 4       # peut etre à modifier, dépend probablement du servo utilisé        
	end = 12.5      # idem
	ratio = (end - start)/180           # coeff directeur pour la conversion angle -> pwm compréhensible par le servo

	angle_as_pwm = angle * ratio

	return start + angle_as_pwm



def ouverture_fermeture(pwm_gpio=12, frequence=50, verbose=False):
	GPIO.setup(pwm_gpio, GPIO.OUT)
	pwm = GPIO.PWM(pwm_gpio, frequence)

	#Init at 0°
	pwm.start(angle_to_percent(0))
	if verbose: print(pwm)
	time.sleep(1)

	pwm.ChangeDutyCycle(angle_to_percent(90))
	if verbose: print(pwm)
	time.sleep(1)


	pwm.ChangeDutyCycle(angle_to_percent(0))
	if verbose: print(pwm)
	time.sleep(1)

	#Close GPIO & cleanup
	pwm.stop()
	GPIO.cleanup()
	return

if __name__=="__main__":
	GPIO.setmode(GPIO.BOARD) #Use Board numerotation mode
	GPIO.setwarnings(False) #Disable warnings
	ouverture_fermeture()
