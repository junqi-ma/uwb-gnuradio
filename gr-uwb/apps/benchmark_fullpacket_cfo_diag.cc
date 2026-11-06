// Isolated CFO-kernel replay. Unchecked variants are diagnostics only.
#include "fullpacket_cfo_diagnostics.h"
#include <gnuradio/uwb/uwb_fullpacket_sic_core.h>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <stdexcept>
#if defined(__SSE__)
#include <xmmintrin.h>
#endif

namespace fs = std::filesystem;
using namespace gr::uwb;
namespace diag = gr::uwb::experiment::cfo_diag;
using cf = std::complex<float>;
using cd = std::complex<double>;

namespace {
std::vector<cf> read_iq(const fs::path& path)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) throw std::runtime_error("cannot read " + path.string());
    const auto bytes = f.tellg();
    if (bytes <= 0 || bytes % sizeof(cf)) throw std::runtime_error("invalid CF32 size");
    std::vector<cf> out(size_t(bytes) / sizeof(cf));
    f.seekg(0); f.read(reinterpret_cast<char*>(out.data()), bytes);
    if (!f) throw std::runtime_error("short CF32 read");
    return out;
}
std::string quote(const std::string& text)
{
    std::ostringstream out; out << '"';
    for (char c : text) {
        if (c == '\\' || c == '"') out << '\\';
        if (c == '\n') out << "\\n"; else out << c;
    }
    out << '"'; return out.str();
}
bool finite(cf value)
{
    return std::isfinite(value.real()) && std::isfinite(value.imag());
}
const char* variant_name(diag::Variant variant)
{
    switch (variant) {
    case diag::Variant::FusedChecked: return "fused_checked";
    case diag::Variant::FusedModelChecked: return "fused_model_checked";
    case diag::Variant::FusedUnchecked: return "fused_unchecked";
    case diag::Variant::SplitChecked: return "split_checked";
    }
    throw std::runtime_error("invalid diagnostic variant");
}
struct Snapshot {
    size_t source = 0;
    uint64_t model_id = 0;
    int64_t absolute_lo = 0, phase_reference = 0;
    double cfo_hz = 0, sample_rate = 0, original_cfo_check_us = 0;
    size_t subnormal_converted = 0, subnormal_model = 0, subnormal_trial = 0;
    std::vector<cd> accum;
    std::vector<cf> iq, expected;
};

std::vector<Snapshot> capture(const fs::path& input_case)
{
    boost::property_tree::ptree pub;
    boost::property_tree::read_json((input_case / "public.json").string(), pub);
    const size_t n = pub.get<size_t>("n");
    if (n < 9000 || n > 2000000 || pub.get<double>("sample_rate") != 998400000. ||
        pub.get<int>("comm_code", 10) != 10 || pub.get<int>("sync_reps", 64) != 64 ||
        pub.get<std::string>("sfd_mode", "4z2") != "4z2" ||
        pub.get<std::string>("pulse_shape", "legacy") != "legacy")
        throw std::runtime_error("unsupported public PHY/context");
    const auto sync = read_iq(input_case / "comm_sync.cf32");
    const auto preamble = read_iq(input_case / "comm_preamble.cf32");
    sic::SicConfig cfg;
    cfg.mode = sic::SicMode::Estimate;
    cfg.roi_lo = pub.get<int64_t>("roi_lo"); cfg.roi_hi = pub.get<int64_t>("roi_hi");
    cfg.observed_end_sample = pub.get<int64_t>("observed_end");
    cfg.detect_threshold = .4; cfg.validate_threshold = .25;
    cfg.online_validation_reps = 0; cfg.k_paths = 4; cfg.coarse_stride = 1;
    cfg.sub_roi_lo = pub.get<int64_t>("consumer_lo", 0);
    cfg.sub_roi_hi = pub.get<int64_t>("consumer_hi", n);
    if (cfg.sub_roi_lo != 0 || cfg.sub_roi_hi != int64_t(n) ||
        cfg.observed_end_sample <= 0 || cfg.observed_end_sample > int64_t(n))
        throw std::runtime_error("diagnostic requires complete consumer context");
    sic::SicScratch estimator;
    std::string error;
    if (!sic::prepare_sic_core(cfg, sync.data(), sync.size(), preamble.data(),
                               preamble.size(), estimator, &error))
        throw std::runtime_error(error);
    sic::SicTemplates templates;
    templates.sync = sync.data(); templates.sync_len = sync.size();
    templates.preamble = preamble.data(); templates.preamble_len = preamble.size();
    demod::Qm35825Profile profile;
    profile.code_index = 10; profile.timing_search_margin = 256;
    profile.timing_coarse_stride = 1;
    demod::core::DemodScratch decode_scratch; decode_scratch.reserve(n);
    mod::HrpModConfig tx;
    tx.code_index = 10; tx.sync_repetitions = 64; tx.sfd_mode = "4z2";
    tx.peak_amplitude = .8f;
    sic::FullPacketScratch full; full.reserve(n);
    if (!full.prepare_prefix_cache()) throw std::runtime_error("prefix preparation failed");
    std::vector<cf> output(n), reconstructed(n);
    std::vector<fs::path> windows;
    for (const auto& entry : fs::directory_iterator(input_case / "windows"))
        if (entry.is_directory() && fs::exists(entry.path() / "raw.cf32"))
            windows.push_back(entry.path());
    std::sort(windows.begin(), windows.end());
    if (windows.empty()) throw std::runtime_error("no source windows");
    if (windows.size() > 16) windows.resize(16);
    std::vector<Snapshot> snapshots;
    for (size_t index = 0; index < windows.size(); ++index) {
        const auto iq = read_iq(windows[index] / "raw.cf32");
        if (iq.size() != n) throw std::runtime_error("raw/public size mismatch");
        sic::SicModel model;
        if (!sic::estimate_model(iq.data(), n, cfg.observed_end_sample, templates,
                                 cfg, estimator, model) || !model.valid)
            throw std::runtime_error("source model failed: " + windows[index].string());
        const auto decoded = demod::core::demodulate_one(
            iq.data(), n, profile, index, model.origin, 0, sync, decode_scratch);
        if (!decoded.payload.fcs_pass)
            throw std::runtime_error("source FCS failed: " + windows[index].string());
        const auto result = sic::fullpacket_cancel(
            iq.data(), n, decoded.phr.phr_bits, decoded.payload.bytes, true, tx,
            model, sync.data(), sync.size(), 0, n, full, output.data(), reconstructed.data());
        if (!result.committed || result.sub_lo < 0 || result.sub_hi <= result.sub_lo ||
            result.sub_hi > int64_t(n))
            throw std::runtime_error(std::string("source reconstruction failed: ") + result.reason);
        Snapshot snap;
        snap.source = index; snap.model_id = model.model_id;
        snap.absolute_lo = result.sub_lo; snap.phase_reference = model.phase_reference_sample;
        snap.cfo_hz = model.cfo_hz; snap.sample_rate = model.sample_rate;
        snap.original_cfo_check_us = result.cfo_check_us;
        snap.accum.assign(full.accum.begin() + result.sub_lo, full.accum.begin() + result.sub_hi);
        snap.iq.assign(iq.begin() + result.sub_lo, iq.begin() + result.sub_hi);
        snap.expected.assign(full.model.begin() + result.sub_lo, full.model.begin() + result.sub_hi);
        for (size_t j = 0; j < snap.iq.size(); ++j) {
            const cf narrowed(snap.accum[j]), delta=snap.iq[j]-snap.expected[j];
            snap.subnormal_converted += (std::fpclassify(narrowed.real())==FP_SUBNORMAL)+(std::fpclassify(narrowed.imag())==FP_SUBNORMAL);
            snap.subnormal_model += (std::fpclassify(snap.expected[j].real())==FP_SUBNORMAL)+(std::fpclassify(snap.expected[j].imag())==FP_SUBNORMAL);
            snap.subnormal_trial += (std::fpclassify(delta.real())==FP_SUBNORMAL)+(std::fpclassify(delta.imag())==FP_SUBNORMAL);
            if (!std::isfinite(snap.accum[j].real()) || !std::isfinite(snap.accum[j].imag()) ||
                !finite(snap.iq[j]) || !finite(snap.expected[j]) ||
                !finite(snap.iq[j] - snap.expected[j]))
                throw std::runtime_error("nonfinite captured source");
        }
        snapshots.push_back(std::move(snap));
    }
    return snapshots;
}
struct Observation { diag::Timings time; double checksum = 0; };
void timings_json(std::ostream& out, const diag::Timings& t)
{
    out << "{\"total_us\":" << t.total_us << ",\"base_us\":" << t.base_us
        << ",\"ramp_us\":" << t.ramp_us << ",\"convert_us\":" << t.convert_us
        << ",\"multiply_us\":" << t.multiply_us << ",\"model_check_us\":" << t.model_check_us
        << ",\"trial_us\":" << t.trial_us << ",\"trial_check_us\":" << t.trial_check_us << '}';
}
double percentile(std::vector<double> sorted, double p)
{
    std::sort(sorted.begin(), sorted.end());
    const double position = p * (sorted.size() - 1);
    const size_t lo = size_t(position), hi = std::min(lo + 1, sorted.size() - 1);
    return sorted[lo] + (sorted[hi] - sorted[lo]) * (position - lo);
}
} // namespace

int main(int argc, char** argv)
try {
    fs::path input_case, outdir;
    size_t observations = 128, rounds = 2, warmup = 8;
    for (int k = 1; k < argc; ++k) {
        const std::string key = argv[k];
        auto next = [&]() { if (++k >= argc) throw std::runtime_error("missing argument"); return std::string(argv[k]); };
        if (key == "--case") input_case = next();
        else if (key == "--out") outdir = next();
        else if (key == "--observations") observations = std::stoull(next());
        else if (key == "--rounds") rounds = std::stoull(next());
        else if (key == "--warmup") warmup = std::stoull(next());
        else throw std::runtime_error("unknown option " + key);
    }
    if (input_case.empty() || outdir.empty() || !observations || !rounds ||
        observations > 100000 || rounds > 100 || warmup > 10000)
        throw std::runtime_error("invalid arguments");
    if (fs::exists(outdir) && !fs::is_empty(outdir))
        throw std::runtime_error("output exists; choose a new run");
    fs::create_directories(outdir);
    unsigned mxcsr = 0;
#if defined(__SSE__)
    mxcsr = _mm_getcsr();
#endif
    const auto snapshots = capture(input_case);
    size_t capacity = 0;
    for (const auto& source : snapshots) capacity = std::max(capacity, source.iq.size());
    std::vector<cd> working_accum(capacity);
    std::vector<cf> working_iq(capacity);
    diag::Scratch scratch; scratch.reserve(capacity);
    const std::array<diag::Variant, 4> variants{diag::Variant::FusedChecked,
        diag::Variant::FusedModelChecked, diag::Variant::FusedUnchecked,
        diag::Variant::SplitChecked};
    std::array<std::vector<Observation>, 4> records;
    for (auto& group : records) group.reserve(observations * rounds);
    std::ofstream log(outdir / "windows.jsonl");
    if (!log) throw std::runtime_error("cannot write observations");
    log << std::setprecision(15);
    auto replay = [&](const Snapshot& source, diag::Variant variant) {
        // Equal hot-buffer policy: copy the selected source before EVERY
        // variant, outside kernel timing. The expected model is never input.
        std::copy(source.accum.begin(), source.accum.end(), working_accum.begin());
        std::copy(source.iq.begin(), source.iq.end(), working_iq.begin());
        diag::Input input;
        input.accum = working_accum.data(); input.iq = working_iq.data();
        input.n = source.iq.size(); input.absolute_lo = source.absolute_lo;
        input.phase_reference = source.phase_reference;
        input.cfo_hz = source.cfo_hz; input.sample_rate = source.sample_rate;
        diag::Timings t;
        const bool success = diag::run(input, variant, scratch, t);
        // Even deliberately unchecked benchmark variants are fully checked
        // AFTER timing; they never participate in production cancellation.
        bool valid = success;
        for (size_t j = 0; j < input.n; ++j)
            valid = valid && finite(scratch.model[j]) && finite(input.iq[j] - scratch.model[j]);
        const bool exact = std::memcmp(scratch.model.data(), source.expected.data(),
                                       input.n * sizeof(cf)) == 0;
        if (!valid || !exact || !std::isfinite(t.total_us) || t.total_us < 0)
            throw std::runtime_error(std::string("kernel replay mismatch: ") + variant_name(variant));
        const double checksum = double(scratch.model[input.n / 3].real()) +
                                double(scratch.model[input.n / 2].imag());
        return Observation{t, checksum};
    };
    for (size_t k = 0; k < warmup; ++k)
        for (const auto variant : variants) replay(snapshots[k % snapshots.size()], variant);
    for (size_t round = 0; round < rounds; ++round) {
        for (size_t index = 0; index < observations; ++index) {
            const size_t source_index = index % snapshots.size();
            auto order = variants;
            if ((index + round) % 2) std::reverse(order.begin(), order.end());
            for (const auto variant : order) {
                const auto record = replay(snapshots[source_index], variant);
                const size_t slot = size_t(variant);
                records.at(slot).push_back(record);
                const auto& source = snapshots[source_index];
                log << "{\"source_window\":" << source.source << ",\"index\":" << index
                    << ",\"round\":" << round + 1 << ",\"variant\":" << quote(variant_name(variant))
                    << ",\"model_id\":" << source.model_id << ",\"n\":" << source.iq.size()
                    << ",\"absolute_lo\":" << source.absolute_lo << ",\"exact_match\":true,\"success\":true,\"times\":";
                timings_json(log, record.time); log << "}\n";
            }
        }
    }
    log.close();
    std::ofstream summary(outdir / "summary.json");
    if (!summary) throw std::runtime_error("cannot write summary");
    summary << std::setprecision(15)
        << "{\"schema\":\"fullpacket-cfo-diagnostic/1\",\"source_case\":" << quote(fs::absolute(input_case).string())
        << ",\"mxcsr_before_capture\":" << mxcsr << ",\"ftz\":" << ((mxcsr & 0x8000)?"true":"false") << ",\"daz\":" << ((mxcsr & 0x40)?"true":"false")
        << ",\"source_count\":" << snapshots.size() << ",\"observations_per_variant_per_round\":" << observations
        << ",\"rounds\":" << rounds << ",\"warmup_per_variant\":" << warmup
        << ",\"buffer_policy\":\"copy compact captured accum and IQ into reusable hot input buffers before every variant; copy outside kernel time\""
        << ",\"scope\":\"isolated CFO kernel, not end-to-end SIC; unchecked modes diagnostic only\""
        << ",\"all_outputs_byte_exact_to_library\":true,\"all_outputs_postvalidated\":true,\"variants\":{";
    bool comma = false;
    for (const auto variant : variants) {
        if (comma) summary << ','; comma = true;
        const auto& group = records[size_t(variant)];
        std::vector<double> values; values.reserve(group.size());
        diag::Timings mean; double checksum = 0;
        for (const auto& record : group) {
            const auto& t = record.time; values.push_back(t.total_us); checksum += record.checksum;
            mean.total_us += t.total_us; mean.base_us += t.base_us; mean.ramp_us += t.ramp_us;
            mean.convert_us += t.convert_us; mean.multiply_us += t.multiply_us;
            mean.model_check_us += t.model_check_us; mean.trial_us += t.trial_us;
            mean.trial_check_us += t.trial_check_us;
        }
        const double count = group.size();
        mean.total_us /= count; mean.base_us /= count; mean.ramp_us /= count;
        mean.convert_us /= count; mean.multiply_us /= count; mean.model_check_us /= count;
        mean.trial_us /= count; mean.trial_check_us /= count;
        summary << quote(variant_name(variant)) << ":{\"n\":" << group.size()
            << ",\"mean_us\":" << mean.total_us << ",\"p50_us\":" << percentile(values, .50)
            << ",\"p95_us\":" << percentile(values, .95) << ",\"p99_us\":" << percentile(values, .99)
            << ",\"checksum\":" << checksum << ",\"mean_times\":";
        timings_json(summary, mean); summary << '}';
    }
    summary << "},\"sources\":[";
    for (size_t j = 0; j < snapshots.size(); ++j) {
        if (j) summary << ',';
        const auto& source = snapshots[j];
        summary << "{\"source_window\":" << source.source << ",\"model_id\":" << source.model_id
            << ",\"n\":" << source.iq.size() << ",\"absolute_lo\":" << source.absolute_lo
            << ",\"phase_reference\":" << source.phase_reference << ",\"cfo_hz\":" << source.cfo_hz
            << ",\"original_setup_cfo_check_us\":" << source.original_cfo_check_us
            << ",\"subnormal_converted_components\":" << source.subnormal_converted
            << ",\"subnormal_model_components\":" << source.subnormal_model
            << ",\"subnormal_trial_components\":" << source.subnormal_trial << '}';
    }
    summary << "]}\n";
    std::cout << "sources=" << snapshots.size() << " observations=" << observations * rounds * variants.size()
              << " all_exact=true\n";
    return 0;
}
catch (const std::exception& error) {
    std::cerr << error.what() << '\n'; return 1;
}
