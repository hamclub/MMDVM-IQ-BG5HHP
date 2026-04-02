This is the source code of the MMDVM firmware that supports D-Star, DMR, System Fusion, P25, NXDN, POCSAG, and FM modes.

It is currently used with the SXceiver Pi hat or similar running on a Raspberry Pi. All of the development work is done on a Pi 4 as there are reports of incompatibilities with the Pi 5.

It connects with the current MMDVM Host via a UDP link. The corresponding entrues at the host end are:

    [Modem]
    Protocol=udp
    ModemAddress=127.0.0.1
    ModemPort=3334
    LocalAddress=127.0.0.1
    LocalPort=3335

This software is licenced under the GPL v2 and is primarily intended for amateur and educational use.
