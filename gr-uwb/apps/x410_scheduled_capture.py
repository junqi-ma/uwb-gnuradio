#!/usr/bin/env python3
"""X410/RFNoC entry for periodic UWB radar-window capture.

The FPGA graph must contain an interpolator that converts the X410 radio rate
737.28 MS/s to 998.4 MS/s (exact rational ratio 65/48).  Host-side GNU Radio
never performs this resampling: it receives fc32 at 998.4 MS/s from an RFNoC
RxStreamer and immediately runs the scheduled extractor.
"""
import argparse
import signal
import sys

RADIO_RATE = 737.28e6
HOST_RATE = 998.4e6
INTERP = 65
DECIM = 48


def parser():
    p = argparse.ArgumentParser()
    p.add_argument("--args", default="type=x4xx", help="UHD device args")
    p.add_argument("--radio-instance", type=int, default=0)
    p.add_argument("--radio-port", type=int, default=0)
    p.add_argument("--upsampler-block", default="0/Upsampler#0",
                   help="RFNoC block ID whose output is exactly 998.4 MS/s")
    p.add_argument("--upsampler-in-port", type=int, default=0)
    p.add_argument("--upsampler-out-port", type=int, default=0)
    p.add_argument("--frequency", type=float, required=True)
    p.add_argument("--gain", type=float, default=20.0)
    p.add_argument("--antenna", default="RX2")
    p.add_argument("--clock-source", default="internal")
    p.add_argument("--time-source", default="internal")
    p.add_argument("--first-packet-sample", type=int, required=True,
                   help="0-based packet start in the 998.4 MS/s host stream")
    p.add_argument("--packet-interval", type=float, default=0.005)
    p.add_argument("--pre-guard", type=int, default=9984)
    p.add_argument("--capture", type=int, default=189696)
    p.add_argument("--post-guard", type=int, default=4096)
    p.add_argument("--output", required=True)
    p.add_argument("--dry-run", action="store_true")
    return p


def validate(a):
    if abs(RADIO_RATE * INTERP / DECIM - HOST_RATE) > 0.5:
        raise ValueError("internal 65/48 RFNoC rate contract is inconsistent")
    if a.first_packet_sample < a.pre_guard:
        raise ValueError("first-packet-sample must be >= pre-guard")
    if a.packet_interval <= 0 or min(a.pre_guard, a.capture, a.post_guard) < 0:
        raise ValueError("invalid schedule/window parameters")


def main():
    a = parser().parse_args()
    validate(a)
    print(f"rate contract: X410 Radio {RADIO_RATE:.0f} S/s -> RFNoC "
          f"{INTERP}/{DECIM} -> GNU Radio {HOST_RATE:.0f} S/s")
    print(f"RFNoC path: Radio#{a.radio_instance}:{a.radio_port} -> "
          f"{a.upsampler_block}:{a.upsampler_in_port}/"
          f"{a.upsampler_out_port} -> RxStreamer(fc32)")
    if a.dry_run:
        return 0

    from gnuradio import gr, uhd, uwb

    graph = uhd.rfnoc_graph(uhd.device_addr(a.args))
    graph.set_clock_source(a.clock_source, 0)
    graph.set_time_source(a.time_source, 0)
    radio = uhd.rfnoc_rx_radio(graph, uhd.device_addr(""), 0, a.radio_instance)
    actual = radio.set_rate(RADIO_RATE)
    if abs(actual - RADIO_RATE) > 0.5:
        raise RuntimeError(f"X410 coerced radio rate to {actual}, expected {RADIO_RATE}")
    radio.set_frequency(a.frequency, a.radio_port)
    radio.set_gain(a.gain, a.radio_port)
    radio.set_antenna(a.antenna, a.radio_port)

    streamer = uhd.rfnoc_rx_streamer(
        graph, 1, uhd.stream_args(cpu_format="fc32", otw_format="sc16"), 1, True)
    graph.connect(radio.get_unique_id(), a.radio_port,
                  a.upsampler_block, a.upsampler_in_port, False)
    graph.connect(a.upsampler_block, a.upsampler_out_port,
                  streamer.get_unique_id(), 0, False)
    graph.commit()

    ext = uwb.scheduled_extractor(
        HOST_RATE, a.packet_interval, a.first_packet_sample,
        a.pre_guard, a.capture, a.post_guard, 16,
        uwb.scheduled_extractor.EmitPolicy.EverySlot, False)
    writer = uwb.packet_writer(a.output, "capture", False)
    tb = gr.top_block("x410_uwb_scheduled_capture")
    tb.connect(streamer, ext)
    tb.msg_connect(ext, "packet", writer, "packet")
    tb.start(1048576)

    stop = False
    def request_stop(_sig, _frame):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    while not stop:
        signal.pause()
    tb.stop(); tb.wait()
    print(f"windows={writer.packets_written()} dropped={writer.packets_dropped()} "
          f"queue_hwm={writer.queue_high_watermark()}")
    return 0 if writer.packets_dropped() == 0 else 2


if __name__ == "__main__":
    sys.exit(main())
