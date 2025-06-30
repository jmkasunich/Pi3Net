# Hardware
While the Pi3Net library doesn't need dedicated hardware, systems using physically long network segments or working in electricially noisy environments benefit from hardware line drivers and recievers that use differential signaling.  And pretty much any multi-board system needs terminals or connectors for the between-board wiring.

This repo contains hardware designs (KiCad 9 format) for boards that might be useful.

## Board 1
Board design 1 uses standard RJ54 type connectors (also known as 8p8c connectors).  These connectors are usually used for ethernet, and CAT5 or better cables with four twisted pairs are readily available in a variety of lengths at very low cost.
This board is intended to connect multiple Picos in a loop or straight-line configuration.  There are two connectors, one goes to the upstream neighbor and one goes downstream.
When fully populated, the board provides the following:
* two half-duplex links that connect all boards
* one full-duplex link connected to the upstream neighbor
* one full-duplex link connected to the downstream neighbor

Each link consumes two Pico pins.  You can choose how many drivers to install based on your needs.  If all you need is a single half-duplex global link, install one driver chip that uses two pins.

All pins that are not used by the communication links are passed through the board.  The intent is to use stacking headers, so that a Pico with this board attached can plug into a breadboard or any other board that would use a "naked" Pico.

