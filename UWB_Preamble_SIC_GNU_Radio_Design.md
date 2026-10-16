# Detection-Triggered Preamble SIC for UWB Sensing–Communication Coexistence

## Goal

Implement a **low-latency, detection-triggered SIC branch in GNU Radio** that cancels interference from a **known UWB preamble** without decoding or reconstructing an entire packet. Support two use cases using the same core algorithm:

| Receiver | Desired signal | Interferer to reconstruct | Cancellation region |
|---|---|---|---|
| Self-Tx/Rx sensing node | Target echoes of sensing preamble | Communication preamble | Overlapping sensing samples, before final sensing CIR estimation |
| Communication node | Communication packet / payload | Sensing preamble | Overlapping payload IQ samples, before demodulation |

## Processing flow

1. **Primary processing and trigger.** Sensing uses its own preamble code to detect echoes and obtain a CIR. Flag a possible preamble collision from anomalous CIR or correlation behavior. Confirm the collision, where feasible, with correlation against the **known interfering preamble code**. At the communication receiver, detect the overlapping sensing preamble via its known code or another reliable timing indicator.
2. **Parallel SIC branch.** Buffer the relevant complex IQ samples. On a trigger, correlate them with a **1016-sample interfering-preamble waveform template** to estimate the interference CIR and timing offset.
3. **Sparse path estimation.** Detect local peaks in the interference CIR, select the **K strongest distinct delay taps**, and estimate their **complex gains** (not just magnitudes). Optionally refine gains with least squares to reduce interference between overlapping paths.
4. **Interference reconstruction.** Generate delayed, complex-scaled copies of the known interfering preamble waveform, including the actual repetition pattern over the overlap window, and sum over K paths. Account for estimated timing, fractional sample delay, and CFO/phase evolution if needed.
5. **IQ-domain cancellation.** Subtract the reconstructed interference from the original, time-aligned complex IQ buffer. Resume the ordinary sensing CIR estimator or communication payload demodulator on cleaned samples. If interference is not confidently detected, bypass SIC unchanged.

## Signal model

Received IQ: `y[n] = s[n] + i[n] + w[n]`, where `s` is the desired signal.

Known-preamble interference:

```text
i[n]       = sum_{k=1..L} h_k * p_I[n - tau_k]
i_hat[n]   = sum_{k in TopK} h_hat_k * p_I[n - tau_hat_k]
y_clean[n] = y[n] - i_hat[n]
```

Here `p_I` is the actual sampled **interfering** preamble waveform (including pulse shaping / filtering and repetitions), `tau_k` is a path delay, and `h_k` is a complex path gain. The **1016 samples** specify the initial template length, **not necessarily the entire preamble duration**. Matched-filter CIR samples must be normalized / calibrated before using them as path gains; alternatively fit the selected template columns to IQ via complex least squares. If CFO is non-negligible, include `exp(j*(2*pi*Delta_f*n/Fs + phi))` during reconstruction.

## Proposed GNU Radio architecture

```text
                             +--> standard preamble correlation / CIR --> collision trigger --+
complex IQ input --> delay / ring buffer                                                | 
              |                                                                           v
              +--> triggered SIC branch: interferer-template correlation --> Top-K paths
                                --> complex waveform reconstruction --> aligned subtraction
                                                                                  |
                                                                                  v
                                                        cleaned IQ --> CIR / payload decoder
```

**Implementation considerations for Codex:**

- Propose GNU Radio block boundaries and decide what belongs in an **Embedded Python Block** (prototype) versus a **C++ OOT block** (low latency).
- Preserve sample alignment: triggered processing needs a buffer covering the preamble detection latency and the complete overlap window. Define trigger transport (stream tags, messages, or shared state), buffer boundaries, sample indices, and synchronization.
- Support configuration of `Fs`, interfering preamble code/template, `K`, peak spacing, trigger threshold, correlation/CIR normalization, overlap length, and optional CFO correction.
- Clarify whether 1016 samples are **IQ waveform samples** or **code/chip samples** at the chosen sampling rate; the reconstruction must use the correctly sampled waveform.
- For communications, cancel only known sensing-preamble contributions that actually overlap payload samples; do not attempt to regenerate unknown sensing payload symbols.
- Allow bypass processing on noncollision frames; specify latency and buffering impact on the normal processing path.

## Minimum validation plan

1. Offline captured/synthetic IQ: verify trigger and cancellation with a single known interfering preamble, then multipath and timing/CFO offsets.
2. Sweep `K = 1, 2, 4, 8, 16` and measure residual interference power, reconstruction NMSE, false-trigger rate, and CPU time / end-to-end latency.
3. Sensing: compare recovered CIR and target detection with/without SIC. Communication: compare payload BER/PER with/without SIC.
4. Stress-test simultaneous/multiple interferers, strong desired-signal leakage into the interferer correlator, and wrong-template triggers.

## Requested output from Codex

Review feasibility in GNU Radio; propose a concrete flowgraph and block interfaces, streaming/buffering strategy, a minimal implementation plan (Python prototype → C++ optimization), and expected latency bottlenecks. Identify assumptions or ambiguities that must be resolved before implementation.
