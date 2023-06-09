DCC_XpressNet_Throttle

This is proven code (10+ years of operation) for simple plug-in XPressnet DCC Throttles.

The goal was to produce a simple throttle:
	- inexpensive;
	- usable without looking at the screen;
	- simple to operate for beginners;
	- turns headlight on/off; on when moving, off after a short delay;
	- extensible.

Although I've prototyped a throttle with LCD display, I really just like these simple ones!

There are some pitfalls (or benefits) with the current code:
	- programmed for ONE locomotive and ONE XPressnet ID;
	- no extra buttons for "flange squeal" or other distractions.

These can be added if desired, after all it's all computer code, and the basics are there.

Hardware:

Arduino. I use 5v Arduino boards. I use a Mega2560 for development, but for the throttles, I just use:

	- Sparkfun Pro Mini 328 - 5V/16MHz. Item # DEV-11113

Interface to XPressnet; we need a couple of things to move the signals between the Arduino and the XPressnet bus:

	- Sparkfun Transceiver Breakout - RS-485. Item# BOB-10124
	- 4 wire cable
	- 5-pin DIN plug (if using the standard LENZ plugs)

Other things:
	- LED and Resistor for power/status indicator;
	- SPST centre off switch for Fwd-stop-reverse;
	- 10k (or thereabouts) linear potentiometer;
	- diode (not necessary, but protects from wiring errors)
	- case, knob, hookup wire, double-sided foam tape, etc.

John.


