# Clock receiver for RFID-MS and MOMO and TAMDL
Software maintenance

DCF77.c

This module implements an German longwave time signal from Mainflingen,
about 25 km south-east of Frankfurt am Main, Germany.
DCF77 is controlled by the Physikalisch-Technische Bundesanstalt (PTB).

Has been used since 2016.

WWVB.c

This module implements an WWVB time signal from radio station near Fort Collins,
Colorado and is operated by the National Institute of Standards and Technology (NIST)

Testing via RFID-MS. 

GPS converter to DCF77.c

The GPS-DCF Converter UTC± can be connected to a DCF77 signal input as an alternative to a DCF77 antenna.
The device receives global UTC via GPS and converts this information into the DCF77 protocol

Testing via RFID-MS.

![My image](https://github.com/peterloes/clock_receiver/blob/master/Getting_Started_Tutorial/4_dcf77_range.jpg)

Reception zone DCF77 (Europe)

![My image](https://github.com/peterloes/clock_receiver/blob/master/Getting_Started_Tutorial/5_wwvb_range.jpg)

Reception zone WWVB (United States)

![My image](https://github.com/peterloes/clock_receiver/blob/master/Getting_Started_Tutorial/1_gps_converter_dcf77.jpg)

Reception zone GPS time signal (Worldwide)
