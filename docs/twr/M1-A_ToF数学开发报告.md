# M1-A 独立 ToF 数学开发报告

日期：2026-09-29
任务单：[OpenCode开发指示_M1-A_ToF数学.md](OpenCode开发指示_M1-A_ToF数学.md)
分支：`feature/uwb-ds-twr`

---

## 1. 结论（先说未完成的那一条）

**M1-A 的代码、QA 与独立验证已完成；但按任务单 §10 的完成判定，M1-A
不能标为完成，因为其中一条为假：「MATLAB 已实跑」。本机没有可用的 MATLAB。**

- 本机 `/usr/local/MATLAB/R2024a/` 存在，但**没有 `bin/matlab` 启动器，也没有
  `bin/glnxa64/MATLAB` 主二进制**，只有支持库与 licence 文件；
- `PATH` 里没有 `matlab`，没有 MATLAB Runtime/MCR，也没有 Octave。

任务单 §2.2 明确禁止把未运行的 `.m` 写成「MATLAB 验证完成」。因此：

- `testdata/twr/generate_tof_oracle.m` 已按任务单写好（可复跑），但**未运行**；
- 检入的 `tof_oracle_vectors.json` 由 `testdata/twr/gen_tof_oracle.py`
  （`fractions.Fraction` 精确实现，**同一个物理模型**）生成；
- 这一点写进了 JSON 的 `provenance` 块、`README_tof_oracle.md` 和本报告；
- **MATLAB 实跑仍是唯一未决项**，补齐后才能按 §10 判 M1-A 完成并进入 M1-B。

我按用户确认的选项执行：Python oracle 继续，MATLAB 标未实跑。

其余完成判定逐条见 §9。

---

## 2. 任务与范围

任务单要求实现**纯 C++、UHD-free、GNU-Radio-free** 的 SS/DS ToF 公式：

- 输入只能是已经过门的 `AdmittedRangingInterval` 与显式 `ClockRatio`；
- 结果是有符号的精确有理数 ticks，**不经过整数 ns、不以 double 秒为中间真值**；
- 用独立 oracle 产生 golden，C++ QA 对照；
- 本阶段结束：公式有、oracle 有、**FSM 仍然没有**。

明确不做（未做）：FSM / fake radio / controller / GNU Radio block / UHD 适配 /
CFO-SFO 估计器 / native 往返 / 首径 ToA / RMARKER 绝对约定 / pybind 大迁移 /
config 业务规则扩张 / PHY 证据级升级。

---

## 3. 交付物

| 交付 | 文件 |
|---|---|
| 数学头（安装） | `gr-uwb/include/gnuradio/uwb/uwb_twr_math.h` |
| C++ QA | `gr-uwb/lib/qa_uwb_twr_math.cc`（13 用例，注册进 CTest） |
| oracle（记录版） | `testdata/twr/generate_tof_oracle.m`（**未实跑**） |
| oracle（生成版） | `testdata/twr/gen_tof_oracle.py`（实跑，产生 golden） |
| golden | `testdata/twr/tof_oracle_vectors.json`（12 向量） |
| oracle 说明 | `testdata/twr/README_tof_oracle.md` |
| 独立验证 | `tools/twr/verify_m1_a_tof.py` |
| 安装验证扩展 | `tools/twr/verify_install_consumer.sh`、`gr-uwb/apps/install_consumer/` |

`uwb_twr_math.h` 已加入安装列表；`uwb_twr_test_output.h` 仍**故意不安装**（QA-only）。

---

## 4. 公式与单位约定（钉死）

命名空间 `gr::uwb::twr`。`k = fA / fB`「A ticks per B tick」，方向固定：
**把 B 域间隔换算到 A 域**。

```
RA = t4A - t1A   DA = t5A - t4A     (A 域)
DB = t3B - t2B   RB = t6B - t3B     (B 域)

SS:  ToF_A = (RA - k*DB) / 2
DS:  ToF_A = (RA*k*RB - DA*k*DB) / (RA + k*RB + DA + k*DB)
```

**DS 的公共单位是 A ticks**（写进头文件注释，测试按此断言）：`rb`、`db` 先乘
`k` 进入 A 域，再套公式。把两端原始 tick 直接相除是 REQ-PROTO-03 明确禁止的。

恒等式（模型可自证）：

```
RA = 2τ + dB, DB = dB/k, DA = dA, RB = (2τ + dA)/k
SS: (2τ + dB - dB)/2 = τ
DS: kRB = 2τ + dA, kDB = dB
    N = (2τ+dB)(2τ+dA) - dA·dB = 2τ(2τ+dA+dB)
    D = 2(2τ+dA+dB)   ->  N/D = τ
```

`computed_at`：SS = `initiator_a`，DS = `responder_b`（REQ-PROTO-05）。注意 DS 的
**单位是 A ticks 而归属于 B**，两者分别显式记录，不会互相冒充。

---

## 5. `ClockRatio` 设计

- `k` 是精确正有理数 `num/den`，构造时约分。负 `k`、分母 0 **拒绝**，绝不用来
  「翻转」公式。
- 来源枚举 `ClockRatioSource`：`nominal_same_clock` / `nominal_rate_ratio` /
  `calibrated` / `estimated`，配 `clock_ratio_source_is_known()`（**无 `default`
  的 switch**）。
- **共同时钟必须显式声明**：`ClockRatio::unity_same_clock(domain_a)`。默认构造的
  `ClockRatio` 是 `provided()==false`，在独立时钟下被拒为 `clock_ratio_missing`
  → `clock_estimate_invalid`。独立时钟**不会**默默填 1。
- `from_nominal_rates(a, b, fA_hz, fB_hz)` 用整数 Hz，不引入 double 舍入；
  1.0 GHz / 998.4 MHz 精确约分为 **625/624**。
- 可选有效期 `valid_from_ticks` / `valid_until_ticks`（A 域，**默认缺席**=不声明）；
  可选不确定度（有理数 ppm，**默认缺席**）。二者在存在时校验（`from<=until`、
  `num>=0`、`den>0`）；§8.4 要求的「往可选字段注入坏值」在 QA 里逐条覆盖。

---

## 6. 失败矩阵覆盖

| ID | 场景 | 结果 |
|---|---|---|
| SS-1 | 共同时钟 `k=1` 对称回复 | `ok`，500 ticks |
| SS-2 | 两个不同 nominal rate（625/624） | `ok`，500；DB 带分数 |
| SS-3a/3b | `k>1` / `k<1`（可实现的最细偏差 ≈30 ppm） | 符号方向正确 |
| SS-4 | 独立时钟缺 ratio | `clock_ratio_missing` → `clock_estimate_invalid` |
| SS-5 | ratio 过期（窗口不含 RA） | `clock_ratio_not_valid_at_time` |
| SS-6 | A/B epoch 不一致 | `clock_ratio_epoch_mismatch`（域错误） |
| SS-7 | 负 ToF | **保留负值** `-500/1`，`negative_tof` |
| SS-8 | 跨 wrap（准入门形成 interval） | 公式不自作模运算，105 ticks |
| DS-1 | 对称回复 `DA=DB` | 与 SS 同一物理 ToF 一致（500） |
| DS-2 | `DA≠DB` 非对称 | 500 |
| DS-3 | 极小 ToF + 长 turnaround，全分数 | `1/7` ticks，无 ns 截断 |
| DS-4 | 四个间隔全 0 | `zero_denominator` |
| DS-5 | 乘法溢出（巨大 k + 大间隔） | `overflow`（**不绕回 double、不静默回绕**） |
| DS-6 | 负 ToF | 保留 `-450/1` |
| DS-7 | `k≠1` 且非对称 | 500（单位换算 load bearing） |
| COM-1 | 域外 `ClockRatioSource` | 构造即拒；predicate fail-closed |
| COM-2 | ns/double 不得成为真值 | 静态断言（含正对照） |
| COM-3 | 裸 `Timestamp` 不能作输入 | 编译期 `static_assert` |

负 ToF 映射到**新增**的 `ExchangeStatus::NegativeTof = 62`（append-only，同步加了
`to_string` / `from_string` / `family` 与 Python 镜像）。其余映射见
`tof_status_to_exchange_status()`；未知 `TofStatus` fail-closed 到 `internal_error`。

---

## 7. Oracle：实际做了什么、没做什么

**做了（实跑）**：`python3 testdata/twr/gen_tof_oracle.py` → 12 个向量。
生成器从物理模型（τ、fA/fB、回复延迟）推导四个间隔，并在写出前用完整公式**自证**
`derived == tau`，否则抛异常、不产出错误 golden。脚本还拒绝任何分母超过 32767
的间隔（DW 时间戳分数域约束）。

向量覆盖：共同时钟、nominal rate ratio、±偏差、分数 ToF、跨 wrap、负 ToF、
对称/非对称 DS、极小 ToF + 长 turnaround、`k≠1` 且非对称。

**没做（未决）**：MATLAB 实跑。原因见 §1。

**次要对照**：`tools/twr/verify_m1_a_tof.py` 用 `fractions.Fraction` 从**原始
端点 ticks**（自己处理回绕）独立重算 SS/DS，并编译一个最小 C++ 探针对照。
它**不**使用 JSON 的 `expected` 字段作为真值——先验证 `expected` 与独立重算一致，
再验证 C++ 与独立重算一致。

---

## 8. QA 与独立验证

- `qa_uwb_twr_math.cc`：13 用例。读 `testdata/twr/tof_oracle_vectors.json`，
  用**真实准入门**（完整 calibration + first-path + corrections）重建每个向量的
  `AdmittedRangingInterval`，断言精确有理数相等（交叉相乘，不要求约分形式一致）。
  失败矩阵用手写期望值，**不**断言两个同源 C++ predicate 相等。
- `tools/twr/verify_m1_a_tof.py`：**4/4**。
- `tools/twr/verify_m0_1_findings.py`：仍 **15/15**（M0.1 回归未坏）。

N07 规则继续适用：新枚举一律 `xxx_is_known()` + **无 `default`** 的 switch，
域检查先于消费该值的 switch。用 `-Wall -Wextra -Wswitch -Wswitch-enum -Werror`
编译包含本头的 TU，**零告警**，证明所有 switch 穷举且无 `default`。

---

## 9. 门禁

| 项 | 结果 |
|---|---|
| 构建 | `cmake . && make -j` 通过，无新增告警 |
| CTest | **53 项 52 通过**；唯一失败仍是历史吞吐 `uwb_qa_uwb_pdu_rational_resampler.cc`，门槛未改；新增 `uwb_qa_uwb_twr_math.cc` 通过 |
| Python | **138 全绿** |
| M0.1 独立验证 | **15/15** |
| M1-A 独立验证 | **4/4** |
| 安装消费者 | 端到端 **ALL OK**（含 `#include <.../uwb_twr_math.h>` 与 `compute_ss_tof` 的 500 例子） |

完成判定 §10 逐条：

- [x] `uwb_twr_math.h` 已安装；外部消费者调用 SS 小例子得到 500 ticks。
- [x] SS：方向/符号/有效期有独立测试；共同时钟与独立时钟分开。
- [x] DS：非对称回复有独立测试；四个间隔先统一单位。
- [x] 负 ToF、分母异常、溢出、过期 ratio、域错误均显式失败且保留有符号原值。
- [ ] **MATLAB 已实跑** —— **否**（本机无可用 MATLAB）。
- [x] 公式路径无整数 ns、无 `double` 秒作为中间真值。
- [x] 未实现 FSM / fake endpoint / block / pybind 大迁移 / config 膨胀。
- [x] 历史吞吐失败仍是唯一允许的 CTest 红灯。
- [ ] `开发状态.md` / `AGENTS.md` 写「M1-A 公式+oracle 完成，协议能力仍无 FSM」——
      已更新为「M1-A 未完成，唯一缺口是 MATLAB 实跑」。
- [x] 报告不声称测距可用、不升级 PHY 证据级、不承诺厘米级。

**因为有一条为假，M1-A 不标完成，也不开 M1-B。**

---

## 10. 明确未做 / 未决

- **未决（阻塞完成判定）**：MATLAB oracle 实跑。需要一台有可用 MATLAB R2024a 的
  机器运行 `matlab -batch "cd('testdata/twr'); generate_tof_oracle"`，重新检入
  `tof_oracle_vectors.json` 并核对。
- 未实现 FSM、fake 双端点、controller、GNU Radio block、UHD 适配、两路 RX、
  timestamp patch。
- 未实现 CFO/SFO **估计**（本阶段只消费 `ClockRatio`）。
- 未实现 native 往返、首径 ToA、RMARKER 绝对约定、reply-delay 测量。
- 未升级 48 行 PHY 白名单的任何证据级（`allows(work_decode, Ranging)` 仍为 0）。
- 未绑定任何 DW1000/DW3000 模组或固件；无空口测试；无 soak。
- 准入门新缺陷：本阶段**未发现**，因此**未改** `admit_ranging_interval()` 的规则。
- 距离只是可选派生字段；未做精度标定，**不声称厘米级**。

---

## 11. 复现

```bash
cd gr-uwb/build && cmake . && make -j"$(nproc)"

env -u LD_LIBRARY_PATH ctest                      # 期望 52/53（唯一失败为历史吞吐）
env -u LD_LIBRARY_PATH ctest -R uwb_qa_uwb_twr_math --output-on-failure
python3 gr-uwb/apps/test_twr_config.py            # 138 OK
python3 tools/twr/verify_m0_1_findings.py         # 15/15
python3 tools/twr/verify_m1_a_tof.py              # 4/4
python3 testdata/twr/gen_tof_oracle.py            # 重新生成 12 向量
tools/twr/verify_install_consumer.sh /tmp/opencode/uwb_install_m1a   # ALL OK

# 补齐唯一未决项（需要可用的 MATLAB）：
# matlab -batch "cd('testdata/twr'); generate_tof_oracle"
```

手工跑测试二进制必须 `env -u LD_LIBRARY_PATH`。安装验证只用临时前缀，不 `sudo install`。

## 12. 文件哈希

```
sha256(uwb_twr_math.h)            = 6ae95f4221818cddecf10a7bcc9d7187b97c4f0df8e848c789707b17cbffca16
sha256(qa_uwb_twr_math.cc)        = 6858fdf13834f3ad7125d4b67f8e09b56404f4d8bbd809f72f2abbb25ba2fbf0
sha256(verify_m1_a_tof.py)        = 8faa896552ff8d1e053f0d2013d651872c970a05284013cbd8e23f53b756b1e3
sha256(tof_oracle_vectors.json)   = 673592679848e6a37a4340316d4986019bc1c19ef31372fb4fb5dfe3a55924ea
```
