# 04: Telemetry test

The same `TEL:` line is emitted every 100 ms on both FTDI USART1 and 3DR USART6.
Send `PING` from either link to receive `PONG`. This verifies crossings, grounds,
baud rate, and that no profiler traffic is contaminating the channel.
