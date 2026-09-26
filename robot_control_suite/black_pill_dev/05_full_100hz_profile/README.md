# 05: Real 100 Hz firmware with timing profile

This project deliberately builds the real `mcu_balance_fusion_wireless` source;
it does not duplicate it. The `P... B... F... I... K... Y... E... C... R... V...`
rows show the 10 ms period, loop body time, free time, and each measured stage.

Flash with wheels lifted. Capture telemetry, then inspect the largest stage and
the lowest `F` (free microseconds) before enabling motors.
