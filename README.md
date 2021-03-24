# Clock receiver for RFID-MS and MOMO and TAMDL
Software maintenance

DCF77.c

This module implements an German longwave time signal from Mainflingen,
about 25 km south-east of Frankfurt am Main, Germany.
DCF77 is controlled by the Physikalisch-Technische Bundesanstalt (PTB).

Has been used since 2016.

WWVB.c

This module implements an WWVB time signal from radio station near Fort Collins,
Colorado and is operated by the National Institute of Standards and Technology (NIST).

Unused potential. Only tested with WWVB Signal Generator. Firmware not reviewed. 

BPC

BPC: China, 68.5 kHz, commercial time signal, started broadcasting 2002-04-25,
data format apparently proprietary, receiver vendors require a commercial licence. 

Proprietary software, also known as non-free software, is computer software
for which the software's publisher or another person reserves some rights
from licenses to use, modify, share modifications, or share the software.

China code radio clock is the Beijing standard time radio signal BPC(frequency 68.5kHz)
transmitted by the radio tower of China National Time Service Center (Shangqiu, Henan Province),
through the clock built-in microprocessor conversion, control the clock moving,
so that the clock display time and Beijing standard time keep accurate and consistent.


GPS converter to DCF77.c

The GPS-DCF Converter UTC± can be connected to a DCF77 signal input as an alternative to a DCF77 antenna.
The device receives global UTC via GPS and converts this information into the DCF77 protocol.

Testing via RFID-MS (2019).

![My image](https://github.com/peterloes/clock_receiver/blob/master/Getting_Started_Tutorial/4_dcf77_range.jpg)

Reception zone U.K. (MSF) and German (DCF77) 

![My image](https://github.com/peterloes/clock_receiver/blob/master/Getting_Started_Tutorial/5_wwvb_range.jpg)

Reception zone U.S. Time Calibration Signal (WWVB)

![My image](https://github.com/peterloes/clock_receiver/blob/master/Getting_Started_Tutorial/7_bpc_range.jpg)

Reception zone Chinese Time Calibration Signal (BPC)

![My image](https://github.com/peterloes/clock_receiver/blob/master/Getting_Started_Tutorial/1_gps_converter_dcf77.jpg)

Reception zone Worldwide with GPS converter to DCF77

https://github.com/peterloes/Clock_receiver_RFID-MS_MOMO_TAMDL/blob/master/Getting_Started_Tutorial/1_gps_poster_overview.pdf

Mainboard:

https://github.com/peterloes/RFID-MS

https://github.com/peterloes/MOMO

https://github.com/peterloes/TAMDL

Manufacture:

https://github.com/peterloes/Clock_receiver_RFID-MS_MOMO_TAMDL/blob/master/Getting_Started_Tutorial/6_Supplier.txt
