# Wireless, no RC

Wireless (3DR radio) firmware/GUI pairs with **no FlySky RC transmitter**
wired in — GUI/bench-tuned over the air, not drivable by a pilot. For the
RC-capable variants (`rc_mcu_ik_wireless`, `rc_balance_fusion_wireless`), see
[`../README.md`](../README.md).

Each subfolder is a self-contained firmware + GUI pair with its own serial
protocol — see its own `README.md` before assuming anything crosses over
between variants (see the repo's `serial-protocol` skill for the general
warning).

| Folder | Purpose |
| :--- | :--- |
| [`mcu_ik_engine_wireless/`](mcu_ik_engine_wireless) | Wireless 3DR GUI tuning module. |
| [`mcu_ik_engine_pretest_wireless/`](mcu_ik_engine_pretest_wireless) | Minimal single-loop wireless balancer; retains a PING/PONG latency probe (`latency_test.py`). |
| [`mcu_balance_fusion_wireless/`](mcu_balance_fusion_wireless) | Sensor-fusion balancer (Stage 2, in progress). |
| [`mcu_pos_wireless/`](mcu_pos_wireless) | Position/encoder-target tuner. |

> **Known discrepancy:** `mcu_balance_fusion_wireless` is grouped here on its
> own name/README ("no RC"), but its `firmware/src/main.cpp` actually has a
> fully wired iBUS RC decoder (drive/steer/arm/crouch) — it's the firmware
> `rc_balance_fusion_wireless` (one level up) was forked from. It's kept here
> because that's what the user asked for when this split was made — don't
> "fix" the placement without checking with them first.
