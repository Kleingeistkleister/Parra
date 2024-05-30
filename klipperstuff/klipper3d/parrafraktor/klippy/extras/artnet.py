#here we will have a class that will be used to receive the artnet data 
# the artnet dmx data will be used to set a target position for a single stepper motor
# the max_speed and max_acceleration for a single stepper motor. this data will be passed to the kinematics class or else.
# the kinematics class will be used to set the target position of the stepper motor and the max_speed and max_acceleration of the stepper motor
# it need to run inside of the reactor thread to be able to set the target position of the stepper motor at any time without any delay
# i use the the parrafraktorpart.py to handle the artnet data.

import stupidArtnet
import parrafraktorpart

#
class ArtnetReceiver():
    def __init__(self) -> None:
        self.parrafraktor = None
        pass
    
    def set_parrafraktor(self, parrafraktor):
        self.parrafraktor = parrafraktor
    
    def set_parrafraktor_dmx(self, dmx):
        self.parrafraktor.set_dmx(dmx)

    def get_artnet_data(self):
        pass
