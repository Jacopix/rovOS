# DVL-A50 protocol documentation: https://docs.waterlinked.com/dvl/dvl-protocol/
int32 id # Transducer number
float64 velocity # Velocity in the direction of the transducer (m/s) 0 if invalid
float64 distance # Distance (parallel to the transducer beam, i.e. not the vertical distance) to the reflecting surface from this transducer (m) -1 if invalid
float64 rssi # Received signal strength indicator: strength of the signal received by this transducer (dBm)
float64 nsd # Noise spectral density: strength of the background noise received by this transducer (dBm)
bool beam_valid
