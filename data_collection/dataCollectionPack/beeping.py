import time
import winsound

def countdown_with_beeps():
    """Three beeps separated by 1 second."""
    low_pitch = 800
    high_pitch = 1200
    duration_low = 300
    duration_high = 500

    time.sleep(1)
    
    winsound.Beep(low_pitch, duration_low)
    
    time.sleep(1)
    
    winsound.Beep(low_pitch, duration_low)

    time.sleep(1)

    winsound.Beep(high_pitch, duration_high)


if __name__ == "__main__":
    countdown_with_beeps()
