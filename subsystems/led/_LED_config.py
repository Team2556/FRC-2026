'''
This is used by the AnimationFactory to easily create animations for CTR CANdle. May need to be updated per LED strip'''

# All LED should start at 8 because 0-7 are reserved for the built in LED lights on the CANdle
# 8 would be the very first LED on the strip and should not change between strips
LED_INDEX_START: int = 8 

# This is the number of individual LED on the strip
# This is used to calculate the end LED index (LED_INDEX_START + LED_LENGTH - 1)
LED_LENGTH: int = 64