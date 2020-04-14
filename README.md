# Serial Port Logger

This project logs serial port activity by acting as a "man in the middle" between two serial ports.

For example to log the activity between APP and DEVICE:
`APP <-> COMA <-> COMB <-> logger <-> COMC <-> DEVICE`

Where there the application being tested connects to COMA, there is a null modem (crossover) connection between COMA and COMB, and the device being monitored is connected to COMC.

Basically this app logs all the data going in each direction and forwards it along.

Messages are framed by a reserved ending character, though this could be changed to a more complex framing, or by using size or timeouts.

This is a test app, and would probably require tweaking based on the use case.

The traffic is written to a binary log with a timestamp, a label for the direction, then the size and contents.
