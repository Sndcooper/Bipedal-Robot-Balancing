# Docs

Reference documentation for the robot's hardware and history. These are
specs/notes, not code — see [`../robot_control_suite/`](../robot_control_suite)
for the active firmware and GUIs.

| Doc | Covers |
| :--- | :--- |
| [`Robot_Specification.md`](Robot_Specification.md) | Physical dimensions, mass, leg geometry, servo IDs, calibration. |
| [`Hardware_Connections.md`](Hardware_Connections.md) | Full pinout, power, USART map, RC channel mapping. |
| [`AX12_Control_Table_Mapping.md`](AX12_Control_Table_Mapping.md) | AX-12+ control table and buffer offsets. |
| [`PAW_Research_Notes.md`](PAW_Research_Notes.md) | Log findings, sensor/actuator/control upgrades, applications, paper topics, roadmap. |
| [`PAW_Future_Roadmap.html`](PAW_Future_Roadmap.html) | The same research roadmap as a styled page (open in a browser). |
| [`Project_History_and_Roadmap.md`](Project_History_and_Roadmap.md) | Project history and phased roadmap (a point-in-time status recap — check current code/commits for what's actually landed since). |

> ⚠️ These docs drift from the code over time (default gains, protocol
> details, which variant is "current"). When a doc and the firmware disagree,
> the firmware you're about to flash wins — treat these as orientation, not
> ground truth.
