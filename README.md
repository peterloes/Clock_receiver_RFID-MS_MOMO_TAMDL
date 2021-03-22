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

BPC.c

China code radio clock is the Beijing standard time radio signal BPC(frequency 68.5kHz)
transmitted by the radio tower of China National Time Service Center (Shangqiu, Henan Province),
through the clock built-in microprocessor conversion, control the clock moving,
so that the clock display time and Beijing standard time keep accurate and consistent.

Unused potential. NOT tested. Firmware not reviewed. 

GPS converter to DCF77.c

The GPS-DCF Converter UTC± can be connected to a DCF77 signal input as an alternative to a DCF77 antenna.
The device receives global UTC via GPS and converts this information into the DCF77 protocol.

Testing via RFID-MS (2019).

![My image](https://github.com/peterloes/clock_receiver/blob/master/Getting_Started_Tutorial/4_dcf77_range.jpg)

Reception zone DCF77 (Europe)

![My image](https://github.com/peterloes/clock_receiver/blob/master/Getting_Started_Tutorial/5_wwvb_range.jpg)

Reception zone WWVB (United States)

![My image](https://github.com/peterloes/clock_receiver/blob/master/Getting_Started_Tutorial/1_gps_converter_dcf77.jpg)

Reception zone GPS time signal (Worldwide)

https://github.com/peterloes/Clock_receiver_RFID-MS_MOMO_TAMDL/blob/master/Getting_Started_Tutorial/1_gps_poster_overview.pdf

Mainboard:

https://github.com/peterloes/RFID-MS

https://github.com/peterloes/MOMO

https://github.com/peterloes/TAMDL

Manufacture:

https://github.com/peterloes/Clock_receiver_RFID-MS_MOMO_TAMDL/blob/master/Getting_Started_Tutorial/6_Supplier.txt
