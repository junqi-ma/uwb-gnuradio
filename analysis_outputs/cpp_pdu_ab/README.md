# C++ PDU EchoTimer A/B 实测

> 设备：X410 CG400（FPGA `CG_400`，491.52 MS/s），`addr=192.168.10.2`，
> TX/RX0 ch0、RX1 ch3。固定：`--preamble-length 128 --pulse-shape minphase
> --gain-tx 50 --gain-rx 60 --cal-delay-native 334 --no-udp`。
> 同一时刻单进程占用 X410。

**重要**：本目录下 `cppab_*`、`cpp500_*` 是**第一轮（SC16 未归一化）**的旧数据，
已被 `round2/` 取代（旧数据 raw CIR/metric 量纲错误，仅保留作对照）。
验收以 `round2/` 为准。

## round2/（整改后，SC16 `UnitRange`）

| 用例 | backend | pulses | wall_s | ok | late | res/est drop | CIR | 备注 |
|---|---|---:|---:|---:|---:|---:|---:|---|
| `200hz_scalecheck` | cpp-pdu | 100 | — | 100 | 0 | 0 | 100 | `metric_mean=0.0539`（旧链 0.0529） |
| `200hz_soak60` | cpp-pdu | 12000 | 60.254 | 12000 | 0 | 0 | 12000 | backlog 平坦 |
| `200hz_soak600` | cpp-pdu | 120000 | 600.250 | 120000 | 0 | 0 | 120000 | 10 min，`peak_tap 17–18`，0 missing/dup |
| `ab/py1..3` | python@`0dbfda5` | 1000 | 5.248 | 998–999 | 1–2 | 0 | — | metric 0.0524–0.0541 |
| `ab/cpp1..3` | cpp-pdu | 1000 | 5.246–5.265 | 1000 | 0 | 0 | — | metric 0.0516–0.0536 |

每个 case 目录含 `summary.json`、`stdout.log`、`run.json`（argv、commit、
设备、artifact SHA-256、backlog 统计）。

## 结论

- **200 Hz fixed X410 验收通过**：60 s 与 10 min soak 均 0 late / 0 overflow /
  0 downstream drop、CIR 连续无缺号、backlog 无正斜率。
- **SC16 契约已统一**：cpp-pdu `metric_mean` 与 Python 链同量级（~0.052），
  旧 `cppab_*` 的 ~1734 是未归一化的历史数据。
- 500 Hz 未验收（见 `../m0_baseline_500hz/`：`0dbfda5` Python 500 Hz 仅
  1227/2500）。
