# Simple Serial

A simple library for communication over serial interfaces. 
It can be used for **microcontroller to microcontroller** communication  or 
**computer to microcontroller** communication.

Currently supported platforms:
* Arduino / C++
* Python

This is a repository for the python version. Find Arduino / C++ version at: 
https://github.com/nplan/simple_serial_arduino

The library supports simple and efficient transfer of data types:
* Integer (signed, 32 bit)
* Floating point (32 bit)
* Strings

Custom data types can be easily added.

## Installation

### Clone

* Clone this repo to your machine

### Setup

* Install using pip:

```shell
pip install {your_download_location}/simple_serial/
```
> Don't forget to use `pip3` if you have multiple python versions installed.

## Usage

Data is sent in packets. Each packet has an ***id*** in range *0 - 255*. The ***id*** 
can be used for identifying the topic of the packet. 

```python
from simple_serial import SimpleSerial, bytes2int, bytes2float, bytes2str

# Open communication
s = SimpleSerial(port="/dev/cu.SLAB_USBtoUART", baud=115200)
s.open()

# Send some data
s.send(id=1, payload=12345)  # Send packet with id 1 containing an integer
s.send(2, 2.345)  # Send packet with id 2 containing a float
s.send(3, "this is a string")  # Send packet containing a string

# Receive data in a loop
while True:
    id, payload = s.read()  # Returns a packet from received buffer
    if id == 123:
        # do something

# Register a callback that is called automatically when a packet with
# certain id is received.
# Payload is passed as argument to the callback function.
s.set_callback(id=30, callback=myFun)


# Convert payload from bytes to desired type
    id, payload = s.read()
    val = bytes2int(payload)  # convert to integer
    val = bytes2float(payload)  # convert to float
    val = bytes2str(payload)  # convert to string
    # Note: there is no way to know of what type the data inside a packet is.
    # You should use id to differentiate between packets.
```

The default payload length in bytes is set to 8. This can be changed by passing argument ```payload_max_len``` to
```SimpleSerial()``` constructor.

## How it works?
Before being sent over serial port, data is framed into packets using using special **flag bytes**:
* *START* Byte - Signalling **start** of packet, *default = ASCII 2 STX*
* *END* Byte - Signalling **end** of packet, *default = ASCII 3 ETX*
* *ESC* Byte - Inserted prior to payload data byte if it happens to have the same byte value 
as one of the flag bytes. *default = ASCII 1 SOH*

A complete packet frame:

BYTE 0| BYTE 1 | BYTE 2 | BYTES 3 ... *N*-1 | BYTE *N*
------|--------|--------|------------- -----|---------
START|LEN|ID|Payload Data Bytes|END

Byte 1 *LEN* indicates the total length of packet in bytes.

When ```SimpleSerial.send()``` functions is called, the payload is framed into a packet
and placed into **send queue**. Packets from this queue are sent over serial port 
using a separate thread.

Serial port is being monitored for incoming packets inside a separate thread.
The thread reads incoming packet frames byte by byte. When a complete packet frame is received, it is placed
into **read queue**. Packets are retrieved from this queue using function ```SimpleSerial.read()```.

The length of send and read queues is set by argument ```queue_max_len``` of ```SimpleSerial()``` constructor.
If a queue is full, packets are discarded (not sent or not received).