#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import UInt8
from std_msgs.msg import Bool
import rospkg
import time
from math import pi

try:
    import RPi.GPIO as GPIO
except ImportError:
    gpio_available = False
else:
    gpio_available = True

    class Buttons(object):

        # Raspi init GPIO 
        PIN_LED_HOTEL = 2 # OUT
        PIN_BUTTON_HOTEL= 3 #IN

        PIN_LED_MILITAIRE = 4
        PIN_BUTTON_MILITAIRE = 14

        #PIN_LED_SEXUEL = None
        #PIN_BUTTON_SEXUEL= None

        #PIN_LED_JUDICIAIRE = None
        #PIN_BUTTON_JUDICIAIRE = None

        #PIN_LED_SOCIAL = None
        #PIN_BUTTON_SOCIAL = None

        #PIN_LED_VOITURE = None
        #PIN_BUTTON_VOITURE = None

        BLINK_DURATION = 1     # blinking on and off

        def __init__(self):

###################################
            self.pub_num = rospy.Publisher("cs_avenir/buttons", ButtonPressed, queue_size=1)
            self.pub_move_in_progress = rospy.Publisher("/move_in_progress", ButtonPressed, queue_size=1)
###################################

            self.move_in_progress = False

            # variable to know if the led is on or off
            # self._hotel_led_on = False
            # self._militaire_led_on = False
            # self._sexuel_led_on = False
            # self._judiciaire_led_on = False
            # self._social_led_on = False
            # self._voiture_led_on = False
            
            # light status : OFF , ON , or BLINK
            #self._hotel_led_status= "OFF"
            #self._militaire_led_status= "OFF"
            # self._sexuel_led_status= "OFF"
            # self._judciaire_led_status= "OFF"
            # self._social_led_status= "OFF"
            # self._voiture_led_status= "OFF"

            self.button_activated = -1

            self.leds=[PIN_LED_JUDICIAIRE,PIN_LED_MILITAIRE,]

            if gpio_available:
                GPIO.setmode(GPIO.BCM)
                #GPIO.setwarnings(False)
                GPIO.setup(self.PIN_BUTTON_HOTEL, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
                GPIO.setup(self.PIN_BUTTON_MILITAIRE, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
                #GPIO.setup(self.PIN_BUTTON_SEXUEL, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
                #GPIO.setup(self.PIN_BUTTON_JUDICIAIRE, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
                #GPIO.setup(self.PIN_BUTTON_SOCIAL, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
                #GPIO.setup(self.PIN_BUTTON_VOITURE, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
                GPIO.setup(self.PIN_LED_HOTEL, GPIO.OUT)
                GPIO.setup(self.PIN_LED_MILITAIRE, GPIO.OUT)
                #GPIO.setup(self.PIN_LED_SEXUEL, GPIO.OUT)
                #GPIO.setup(self.PIN_LED_JUDICIAIRE, GPIO.OUT)
                #GPIO.setup(self.PIN_LED_SOCIAL, GPIO.OUT)
                #GPIO.setup(self.PIN_LED_VOITURE, GPIO.OUT)

            else:
                rospy.logwarn("Node hasn't found the GPIO, buttons will not work")
            rospy.loginfo("Buttons node is started!")
        
       def callback_move(self,data):
           self.move_in_progress = data.data


        def button_update(self):
            if gpio_available:

                if GPIO.input(PIN_BUTTON_HOTEL)==0:                        # attend que bouton soit appuyer
                    self.button_activated=0
                    print 'btn hotel push'
                elif GPIO.input(PIN_BUTTON_MILITAIRE)==0:
                    self.button_activated=1
                    print 'btn militaire push'
                else:
                    self.button_activated=-1    # aucun appuie
             
                if self.button_activated != -1:
                    self.pub_num.publish(self.button_activated)
                    #########################################
                    # publish true when the move begin
                    pub_move_in_progress.pubish(1)
                    #########################################
                  
        
        def all_led_on(self):
            for pin_led in self.leds:
                GPIO.output(pin_led,true)
                time.sleep(0.3)
        
        def all_led_off(self):
            for pin_led in self.leds:
                GPIO.output(pin_led,false)

        def blink_led(self):
            GPIO.output(self.leds[self.button_activated],true)
            time.sleep(BLINK_DURATION)
            GPIO.output(self.leds[self.button_activated],false)
            time.sleep(BLINK_DURATION)



        def led_update(self):

            if self.button_activated == -1:
                self.all_led_on()
            
            else:
                self.blink_led()
            
        
        def run(self):
            rate = rospy.Rate(10)
            while not rospy.is_shutdown():
                self.button_update()
                while self.move_in_progress:
                    self.led_update()
                    rospy.Subscriber("/move_in_progress", Bool, self.callback_move)
    
                rate.sleep()

if __name__ == '__main__':
    rospy.init_node("cs_avenir_buttons")
    buttons = Buttons()
    buttons.run()