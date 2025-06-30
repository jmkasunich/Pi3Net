![Image](https://github.com/user-attachments/assets/b68b4a29-d54e-463c-ac13-183903618e21)

# Pi3Net

Lightweight short-range networking for multiple Raspberry Pi Pico MCUs.

This project came about because I needed to connect 14 Raspberry Pi Pico boards together for a moving sculpture project.  As I designed my networking system I realised that it might be useful to others so I broke it out into its own project.

The name **Pi3Net** is short for **Pi Pico PIO Networking.**
* **Pi** is Raspberry Pi
* **Pico** is the Pico board, using the RP2040 MCU.  Unlike the regular Pi which runs Linux, the Pico is a lightweight MCU that usually runs firmware without an operating system.
* **PIO** is the RP2040's Programmable I/O unit, a dedicated co-processor that offloads bit-banging type I/O from the main processor and lets you develop custom interfaces.
* **Networking** is communication between a group of MCUs.

Although in some cases Pi3Net uses ethernet-style RJ45 connectors, it is NOT ethernet.  It is a very lightweight interface, implementing only a couple layers of the traditional networking model.  There are no IP addresses, no DHCP, no routers, sockets, etc.  It is just a way to quickly and efficiently send messages between MCUs.  The physical layer can be as simple as a single wire (plus ground), which works well if both MCUs are on the same board.  If the MCUs are further apart, on different boards, or in a noisy environment, you can add line drivers and recievers using RS-422 or RS-485 differential signals for better data integrity.  This project includes an open-source PC board design with line drivers and connectors to make it simple.
