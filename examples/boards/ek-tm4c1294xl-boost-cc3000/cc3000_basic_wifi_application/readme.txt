CC3000 Basic WiFi Example

This is a basic WiFi application for the CC3000 BoosterPack. This
application is a command line wrapper for various functions that the
CC3000 can provide. Please refer to the CC3000 wiki at
http://processors.wiki.ti.com/index.php/CC3000 for more information on
the commands provided.

To see available commands type ``help'' at the serial terminal prompt.
The terminal is connected in 8-N-1 mode at 115200 baud.

This example defaults to using BoosterPack2. If you would like to use
BoosterPack1 instead please change the define CC3000_USE_BOOSTERPACK2 in
the project settings to CC3000_USE_BOOSTERPACK1 and rebuild.

To use this example you must first connect to an existing unencrypted
wireless network. This can be done by using the ``smartconfig'' command
with the associated smartphone application. Alternatively, the connection
can be made manually by using the 'connect' command. Once connected you can
do any of the following.

Configure an IP address:

<ol>
<li>To use DHCP to allocate a dynamic IP address ``ipconfig'' or
``ipconfig 0 0 0'' or,
<li>To allocate a static IP address use ``ipconfig a.b.c.d'' where
``a.b.c.d'' is the required, dotted-decimal format address.
</ol>

Send and receive UDP data:

<ol>
<li>Open a UDP socket ``socketopen UDP''.
<li>Bind the socket to a local port ``bind 8080''.
<li>Send or receive data ``senddata 192.168.1.101 8080 helloworld'' or
``receivedata''.  In the senddata case, the provided parameters identify
the IP address of the remote host and the remote port number to which the
data is to be sent.
</ol>

Send and receive TCP data:

<ol>
<li>Open a TCP socket ``socketopen TCP''.
<li>Bind the socket to a local port ``bind 8080''.
<li>Send a request to the remote server ``senddata 192.168.1.101 8080
helloworld''.  On the first ``senddata'' after opening the socket, the
socket is connected to the specified remote host and port.  On further
``senddata'' requests, the remote address and port are ignored and the
existing connection is used.
<li>Receive data from the remote server ``receivedata''.
</ol>

Note that, in the current implementation, the application only supports
acting as a TCP client.  The CC3000 also supports incoming connections
as required to operate as a TCP server but this example does not yet
include support for this feature.

Send mDNS advertisement:

<ol>
<li>``mdnsadvertise cc3000''
</ol>

Close the open socket:

<ol>
<li>``socketclose''
</ol>

Disconnect from network:

<ol>
<li>``disconnect''
</ol>

Reset the CC3000:

<ol>
<li>``resetcc3000''
</ol>

Delete connection policy:

This deletes the connection policy from CC3000 memory so that the device
won't auto connect whenever it is reset in future.

<ol>
<li>``deletepolicy''
</ol>

-------------------------------------------------------------------------------

Copyright (c) 2014-2015 Texas Instruments Incorporated.  All rights reserved.
Software License Agreement

Texas Instruments (TI) is supplying this software for use solely and
exclusively on TI's microcontroller products. The software is owned by
TI and/or its suppliers, and is protected under applicable copyright
laws. You may not combine this software with "viral" open-source
software in order to form a larger program.

THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
DAMAGES, FOR ANY REASON WHATSOEVER.

This is part of revision 2.1.1.71 of the EK-TM4C1294XL Firmware Package.
