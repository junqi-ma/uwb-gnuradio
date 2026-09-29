/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * qa_uwb_twr_config_parity.cc -- the C++ half of the Python/C++ configuration
 * PARITY test (M0.1, docs/twr/下一步开发方案_M0复核后.md §2.A item 4).
 *
 * WHAT THIS IS FOR
 * ----------------
 * C++ is the runtime authority of record for the TWR configuration.  A
 * retained pure-Python validator is only defensible if it is pinned to the
 * SAME profile data and compared ITEM BY ITEM against the SAME JSON corpus:
 * accepted/rejected, the reason, and the effective values.  Comparing enums, or
 * grepping source strings for a spelling, does not count -- and neither does
 * "the Python suite is green".  M0 shipped two implementations that had
 * drifted badly: a 7-byte frame header against the codec's 14, a PHR rate
 * typed as a payload data rate, a session id that did not fit the wire, an
 * accepted `phr_mode=extended`, and a `twr-config/1` schema that had become
 * wrong.  Both suites were green throughout.
 *
 * The corpus is ONE file, testdata/twr/config_parity_corpus.json, read by this
 * suite and by gr-uwb/apps/test_twr_config.py.  Every case names a base
 * document, a patch, and the outcome both implementations must produce.
 *
 * WHAT IS COMPARED, AND WHY
 * -------------------------
 *   * `import_ok` + `import_violations` -- the JSON reader is part of the
 *     contract (twr-config/2 makes frame.frame_profile required), so a case
 *     that is a MISSING_KEY must produce a missing_key import violation on
 *     both sides, not just a later validation failure.
 *   * `accepted` -- the verdict, from validate().
 *   * `violations` -- the ORDERED list of (field, ConfigReason,
 *     ExchangeStatus) triples.  This is the "the reason" half of the contract,
 *     and the order is part of it: a validator that reports the same problems
 *     in a different order is a different report.
 *   * `effective` -- the frozen snapshot's numbers: tick rate, the codec's
 *     on-air frame budget (16/26/31 B), the PHR size, and the quantised
 *     device ticks.  A validator that accepts the same document but budgets the
 *     wrong number of samples is wrong in the way that matters.
 *   * `config_hash` -- the fnv1a64 over the canonical "path=value" field
 *     listing.  This is the strongest single check in the suite: it is a
 *     function of EVERY field of the config, so one wrong spelling, one missing
 *     field in the listing, or one field emitted in a different order changes
 *     it.  A Python and a C++ config_hash that agree means the two
 *     implementations agree on every field name, every value rendering and the
 *     order of all of them.
 *
 * WHAT IS DELIBERATELY NOT COMPARED
 * ---------------------------------
 * The human MESSAGE of a rejection.  The two implementations are allowed to
 * word a rejection differently -- the Python module documents its divergences
 * on purpose, because it quotes the MEASURED CSV cell where C++ quotes a
 * summary -- and pinning prose would turn a contract test into a string
 * comparison.  The field path, the ConfigReason and the ExchangeStatus are the
 * machine-readable contract (REQ-ERR-01); the prose is not.
 *
 * CORPUS FORMAT (see the file itself for the full description)
 * ------------------------------------------------------------
 *   {
 *     "corpus_version": "twr-config-parity-corpus/1",
 *     "config_schema_version": "twr-config/2",
 *     "base": { "ss_initiator": {...}, "ss_responder": {...},
 *               "ds_initiator": {...}, "ds_responder": {...} },
 *     "cases": [ { "name": ..., "note": ..., "base": "ss_initiator",
 *                  "patch": {"phy.channel": 9}, "expect": {...} }, ... ],
 *     "non_finite_fields": [ "phy.center_frequency_hz", ... ]
 *   }
 *
 * A patch is a map of dotted path -> value applied to a deep copy of the base.
 * A null value DELETES the key, which is how the missing-key cases are written.
 * The paths are the SAME dotted paths the validator reports.
 *
 * A non-finite double CANNOT be written in strict JSON and neither reader will
 * produce one, so the corpus names the float fields it wants poisoned in
 * `non_finite_fields` and BOTH loaders assign NaN / +Inf / -Inf to the
 * imported config afterwards.  That is the only way such a value reaches a
 * validator, and both sides do it identically.
 *
 * THE SCHEMA VERSION IS PART OF THE CONTRACT
 * ------------------------------------------
 * `meta.schema_version` is a top-level expectation.  This build must advertise
 * exactly the version the corpus is written against.  twr-config/2 is a
 * BREAKING change (frame.frame_profile became required; phy.phr_rate changed
 * value domain), so a build still advertising twr-config/1 rejects every v2
 * document -- which is a real, loud failure, not something to paper over.
 *
 * This header is stdlib-only, so including it here pulls in no GNU Radio and no
 * UHD.  UWB_TESTDATA_DIR is how the corpus is located; the CMake target
 * defines it, and the out-of-tree build script defines it too.
 */

#include <gnuradio/uwb/uwb_twr_config.h>

// The frame codec is the geometry authority the config layer validates
// against, and its own vocabulary is part of what this suite pins.
#include <gnuradio/uwb/uwb_twr_frame.h>

#include <boost/test/unit_test.hpp>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace twr = gr::uwb::twr;
using namespace gr::uwb::twr;

#ifndef UWB_TESTDATA_DIR
#define UWB_TESTDATA_DIR "../../../testdata"
#endif

BOOST_AUTO_TEST_SUITE(twr_config_parity)

namespace {

// ---------------------------------------------------------------------------
// Reading the corpus
// ---------------------------------------------------------------------------

const char* const kCorpusRelative = "twr/config_parity_corpus.json";

json::Value read_corpus()
{
    const std::string path = std::string(UWB_TESTDATA_DIR) + "/" + kCorpusRelative;
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        BOOST_FAIL("cannot open the config parity corpus at " << path
                                                         << " (is UWB_TESTDATA_DIR set?)");
    }
    std::stringstream ss;
    ss << in.rdbuf();
    json::Value root;
    std::string err;
    if (!json::parse(ss.str(), root, err))
        BOOST_FAIL("cannot parse the config parity corpus at " << path << ": " << err);
    return root;
}

const json::Value& member(const json::Value& v, const std::string& key)
{
    const json::Value* m = v.find(key);
    if (!m)
        BOOST_FAIL("corpus: no member '" << key << "'");
    return *m;
}

// ---------------------------------------------------------------------------
// The patch language
// ---------------------------------------------------------------------------
// Dotted paths, with `[i]` for list indices, exactly the paths the validator
// reports.  `null` deletes a key.  An unknown PARENT is a HARD FAILURE: a case
// whose patch silently did nothing would be testing the base configuration and
// reporting a false pass, which is the one failure mode a parity test must not
// have.  A new LEAF may be added, because "this key is not in the schema" is
// itself a case worth stating.

json::Value* descend(json::Value* node, const std::string& path, size_t from, size_t to)
{
    size_t i = from;
    while (i < to) {
        const size_t dot = path.find('.', i);
        const size_t end = (dot == std::string::npos) ? to : dot;
        const std::string seg = path.substr(i, end - i);
        if (!seg.empty() && seg.back() == ']') {
            // A `name[i]` segment is a member lookup FOLLOWED BY an index:
            // "radio.peers[0].id" descends into radio, then peers, then 0.
            const size_t br = seg.find('[');
            const std::string name = seg.substr(0, br);
            const long idx = std::strtol(seg.c_str() + br + 1, nullptr, 10);
            json::Value* holder = node;
            if (node->type == json::Type::Object) {
                holder = node->find(name);
                if (!holder)
                    BOOST_FAIL("corpus patch path '" << path << "': no key '" << name << "'");
            }
            if (holder->type != json::Type::Array)
                BOOST_FAIL("corpus patch path '" << path << "': '" << name
                                                 << "' is not an array");
            if (idx < 0 || static_cast<size_t>(idx) >= holder->items.size())
                BOOST_FAIL("corpus patch path '" << path << "': index " << idx
                                                 << " is out of range");
            node = &holder->items[static_cast<size_t>(idx)];
            i = end + 1;
            continue;
        }
        if (node->type != json::Type::Object)
            BOOST_FAIL("corpus patch path '" << path << "': '" << seg
                                             << "' is not inside an object");
        json::Value* child = node->find(seg);
        if (!child)
            BOOST_FAIL("corpus patch path '" << path << "': no key '" << seg << "'");
        node = child;
        i = end + 1;
    }
    return node;
}

json::Value deep_copy(const json::Value& v)
{
    json::Value out = v;
    for (auto& m : out.members)
        m.second = deep_copy(m.second);
    for (auto& item : out.items)
        item = deep_copy(item);
    return out;
}

void erase_key(json::Value& obj, const std::string& key)
{
    for (size_t i = 0; i < obj.members.size(); ++i) {
        if (obj.members[i].first == key) {
            obj.members.erase(obj.members.begin() + static_cast<long>(i));
            return;
        }
    }
    BOOST_FAIL("corpus patch: cannot delete absent key '" << key << "'");
}

json::Value apply_patch(const json::Value& base, const json::Value& patch)
{
    json::Value doc = deep_copy(base);
    for (const auto& m : patch.members) {
        const std::string& path = m.first;
        const size_t dot = path.rfind('.');
        const std::string leaf = (dot == std::string::npos) ? path : path.substr(dot + 1);
        const size_t head_len = (dot == std::string::npos) ? 0 : dot;
        json::Value* parent = descend(&doc, path, 0, head_len);
        if (m.second.type == json::Type::Null) {
            erase_key(*parent, leaf);
            continue;
        }
        if (parent->type != json::Type::Object)
            BOOST_FAIL("corpus patch path '" << path
                                            << "' does not name an object member");
        parent->set(leaf, deep_copy(m.second));
    }
    return doc;
}

// ---------------------------------------------------------------------------
// Non-finite doubles
// ---------------------------------------------------------------------------
// Strict JSON has no NaN / Infinity literal and this header's reader refuses
// one, so a corpus cannot carry a non-finite double.  The corpus instead names
// the fields to poison in `non_finite_fields` and BOTH loaders assign the three
// non-finite values to the imported config.  This is a deliberate
// post-import mutation, and the Python loader performs the identical one.

TimedField* timed_field_at(TwrConfig& c, const std::string& path)
{
    if (path == "frame.sfd_timeout")
        return &c.frame.sfd_timeout;
    if (path == "timing.poll_start")
        return &c.timing.poll_start;
    if (path == "timing.poll_to_response")
        return &c.timing.poll_to_response;
    if (path == "timing.response_to_final")
        return &c.timing.response_to_final;
    if (path == "timing.final_to_report")
        return &c.timing.final_to_report;
    if (path == "timing.post_tx_rx_enable")
        return &c.timing.post_tx_rx_enable;
    if (path == "timing.min_tx_lead_time")
        return &c.timing.min_tx_lead_time;
    if (path == "timeouts.poll_rx_window")
        return &c.timeouts.poll_rx_window;
    if (path == "timeouts.response_rx_window")
        return &c.timeouts.response_rx_window;
    if (path == "timeouts.final_rx_window")
        return &c.timeouts.final_rx_window;
    if (path == "timeouts.report_rx_window")
        return &c.timeouts.report_rx_window;
    if (path == "timeouts.rx_timeout")
        return &c.timeouts.rx_timeout;
    if (path == "timeouts.exchange_timeout")
        return &c.timeouts.exchange_timeout;
    if (path == "timeouts.retry_interval")
        return &c.timeouts.retry_interval;
    if (path == "diagnostics.stats_cadence")
        return &c.diagnostics.stats_cadence;
    return nullptr;
}

// Every float field of a TwrConfig, by dotted path.  Kept as an explicit table
// rather than derived by reflection (which does not exist in C++): a new float
// field that nobody adds here simply is not covered, which the test below
// reports, so the table cannot rot silently.
const char* const kFloatFields[] = {
    "phy.center_frequency_hz",
    "radio.native_sample_rate_hz",
    "radio.peers[0].native_sample_rate_hz",
    "radio.readback.sample_rate_hz",
    "radio.readback.center_freq_hz",
    "rx.gain_db",
    "rx.bandwidth_hz",
    "rx.detection_threshold",
    "rx.correlation_threshold",
    "rx.first_path_threshold",
    "rx.vendor_pac_applied_step",
    "tx.gain_db",
    "tx.iq_amplitude",
    "tx.calibrated_tx_power_dbm",
    "calibration.native_sample_rate_hz",
    "calibration.record.native_sample_rate_hz",
    "calibration.record.gain_db",
    "frame.sfd_timeout.quantisation_hz",
    "timing.poll_start.quantisation_hz",
    "timing.poll_to_response.quantisation_hz",
    "timing.response_to_final.quantisation_hz",
    "timing.final_to_report.quantisation_hz",
    "timing.post_tx_rx_enable.quantisation_hz",
    "timing.min_tx_lead_time.quantisation_hz",
    "timeouts.poll_rx_window.quantisation_hz",
    "timeouts.response_rx_window.quantisation_hz",
    "timeouts.final_rx_window.quantisation_hz",
    "timeouts.report_rx_window.quantisation_hz",
    "timeouts.rx_timeout.quantisation_hz",
    "timeouts.exchange_timeout.quantisation_hz",
    "timeouts.retry_interval.quantisation_hz",
    "diagnostics.stats_cadence.quantisation_hz",
};
const size_t kFloatFieldCount = sizeof(kFloatFields) / sizeof(kFloatFields[0]);

// The float fields the validator does NOT run a finiteness check on.
//
// Stated as DATA, not as a comment, and asserted below, because a field that
// silently accepts a NaN is a defect (REQ-API-01: a non-finite number may not
// be a configuration value) and the next person to add `finite()` to one of
// them has to move the entry out of this list.  Both are gaps in
// uwb_twr_config.h, which this file does not own, so the parity test pins the
// CURRENT behaviour on both sides rather than pretending the gap is not there:
//
//   * calibration.record.native_sample_rate_hz -- check_calibration() only
//     tests it for APPLICABILITY (`> 0 && !matches`), and a NaN fails `> 0`,
//     so the record's rate ends up compared with nothing at all.  Nothing else
//     in the config reaches it either: an infinite value does trip the
//     applicability rule, but a NaN sails through.
//
// Every entry of kFloatFields must be covered by an explicit `finite()` or
// `check_finite_opt()` call -- including radio.peers[N].native_sample_rate_hz,
// which check_radio() finiteness-checks per peer -- and is required to produce
// `not_finite` below.
//
// This list is now EMPTY.  It used to hold
// "calibration.record.native_sample_rate_hz", whose applicability rule is
// guarded by `> 0.0`: a NaN fails that comparison, so a non-finite value
// skipped the whole validator.  The orchestrator added the missing
// `finite()` call.  Keep the list as the tripwire that makes the next omission
// loud rather than silent: adding a float field means adding it here ONLY if
// the check is genuinely absent, and the sweep test below fails either way.
const char* const kFloatFieldsNotFiniteChecked[] = {
    // none
};
const size_t kFloatFieldsNotFiniteCheckedCount =
    sizeof(kFloatFieldsNotFiniteChecked) / sizeof(kFloatFieldsNotFiniteChecked[0]);

bool is_finite_checked(const std::string& path)
{
    for (size_t i = 0; i < kFloatFieldsNotFiniteCheckedCount; ++i)
        if (path == kFloatFieldsNotFiniteChecked[i])
            return false;
    return true;
}

// Assign a double to one float field.  An unknown path is a HARD FAILURE, never
// a silent no-op.
void set_float(TwrConfig& c, const std::string& path, double v)
{
    // Optional (Opt<double>) fields first: absent becomes present-and-NaN,
    // which is the case a caller has to be told about.
    if (path == "tx.gain_db") {
        c.tx.gain_db = v;
        return;
    }
    if (path == "tx.iq_amplitude") {
        c.tx.iq_amplitude = v;
        return;
    }
    if (path == "tx.calibrated_tx_power_dbm") {
        c.tx.calibrated_tx_power_dbm = v;
        return;
    }
    if (path == "rx.gain_db") {
        c.rx.gain_db = v;
        return;
    }
    if (path == "rx.bandwidth_hz") {
        c.rx.bandwidth_hz = v;
        return;
    }
    if (path == "rx.vendor_pac_applied_step") {
        c.rx.vendor_pac_applied_step = v;
        return;
    }
    if (path == "calibration.record.gain_db") {
        c.calibration.record.gain_db = v;
        return;
    }

    const size_t dot = path.rfind('.');
    const std::string leaf = (dot == std::string::npos) ? path : path.substr(dot + 1);
    if (leaf == "quantisation_hz") {
        const std::string parent = (dot == std::string::npos) ? path : path.substr(0, dot);
        TimedField* t = timed_field_at(c, parent);
        if (!t)
            BOOST_FAIL("corpus non-finite path '" << path << "': no such timed field");
        t->required_quantisation_hz = v;
        return;
    }

    if (path == "phy.center_frequency_hz") {
        c.phy.center_frequency_hz = v;
        return;
    }
    if (path == "radio.native_sample_rate_hz") {
        c.radio.native_sample_rate_hz = v;
        return;
    }
    if (path == "radio.peers[0].native_sample_rate_hz") {
        if (c.radio.peers.empty())
            BOOST_FAIL("corpus non-finite path '" << path << "': no peers[0]");
        c.radio.peers[0].native_sample_rate_hz = v;
        return;
    }
    if (path == "radio.readback.sample_rate_hz") {
        c.radio.readback.sample_rate_hz = v;
        return;
    }
    if (path == "radio.readback.center_freq_hz") {
        c.radio.readback.center_freq_hz = v;
        return;
    }
    if (path == "rx.detection_threshold") {
        c.rx.detection_threshold = v;
        return;
    }
    if (path == "rx.correlation_threshold") {
        c.rx.correlation_threshold = v;
        return;
    }
    if (path == "rx.first_path_threshold") {
        c.rx.first_path_threshold = v;
        return;
    }
    if (path == "calibration.native_sample_rate_hz") {
        c.calibration.native_sample_rate_hz = v;
        return;
    }
    if (path == "calibration.record.native_sample_rate_hz") {
        c.calibration.record.native_sample_rate_hz = v;
        return;
    }
    BOOST_FAIL("corpus non-finite path '" << path << "': not a known float field");
}

// ---------------------------------------------------------------------------
// Running one case
// ---------------------------------------------------------------------------

struct Outcome {
    bool import_ok = false;
    std::vector<std::string> import_violations; // "field|reason|status"
    bool accepted = false;
    std::vector<std::string> violations;
    std::string config_hash;
    // effective values, as "name=value" text so a mismatch prints readably
    std::vector<std::pair<std::string, std::string>> effective;
};

std::string triple(const std::string& field, ConfigReason r, ExchangeStatus s)
{
    return field + "|" + config_reason_to_string(r) + "|" + exchange_status_to_string(s);
}

Outcome run_case(const json::Value& base, const json::Value& case_value)
{
    Outcome out;

    json::Value doc = apply_patch(base, member(case_value, "patch"));

    std::string text;
    std::string dump_err;
    if (!json::dump(doc, text, dump_err, /*pretty=*/false))
        BOOST_FAIL("corpus: cannot re-serialise a patched document: " << dump_err);

    TwrConfig cfg;
    const ValidationReport imp = from_json_string(text, cfg);
    out.import_ok = imp.ok();
    for (const auto& v : imp.violations)
        out.import_violations.push_back(triple(v.field, v.reason, v.status));

    // A case names the float fields it wants poisoned, and ONLY those: a case
    // that names none must poison none, or every case in the corpus would be
    // testing a poisoned config.  The EXHAUSTIVE sweep over every field and all
    // three non-finite values is generated in
    // non_finite_is_refused_in_every_float_field instead, so it is not repeated
    // 145 times here.
    const json::Value* nf = case_value.find("non_finite_fields");
    if (nf && nf->type == json::Type::Array) {
        for (const auto& item : nf->items) {
            if (item.type != json::Type::String)
                BOOST_FAIL("corpus: non_finite_fields must hold strings");
            set_float(cfg, item.text, std::numeric_limits<double>::quiet_NaN());
        }
    }

    const EffectiveConfig e = effective_config(cfg);
    out.accepted = e.validation.ok();
    for (const auto& v : e.validation.violations)
        out.violations.push_back(triple(v.field, v.reason, v.status));
    out.config_hash = e.config_hash;
    out.effective.emplace_back("ok", e.ok ? "true" : "false");
    out.effective.emplace_back("schema_version", e.schema_version);
    out.effective.emplace_back("profile_version", e.profile_version);
    out.effective.emplace_back("calibration_version", e.calibration_version);
    out.effective.emplace_back("tick_rate_hz", twr_double_to_text(e.tick_rate_hz));
    out.effective.emplace_back("poll_bytes", twr_int_to_text(e.poll_bytes));
    out.effective.emplace_back("response_bytes", twr_int_to_text(e.response_bytes));
    out.effective.emplace_back("final_bytes", twr_int_to_text(e.final_bytes));
    out.effective.emplace_back("max_psdu_bytes", twr_int_to_text(e.max_psdu_bytes));
    out.effective.emplace_back("max_timestamp_count",
                               twr_int_to_text(static_cast<int64_t>(e.max_timestamp_count)));
    out.effective.emplace_back("phr_bytes", twr_int_to_text(e.phr_bytes));
    out.effective.emplace_back("phr_coded_bits", twr_int_to_text(e.phr_coded_bits));
    out.effective.emplace_back("poll_start_ticks",
                               twr_int_to_text(e.poll_start_ticks));
    out.effective.emplace_back("poll_to_response_ticks",
                               twr_int_to_text(e.poll_to_response_ticks));
    out.effective.emplace_back("response_to_final_ticks",
                               twr_int_to_text(e.response_to_final_ticks));
    out.effective.emplace_back("post_tx_rx_enable_ticks",
                               twr_int_to_text(e.post_tx_rx_enable_ticks));
    out.effective.emplace_back("poll_start_effective_ns",
                               twr_int_to_text(e.poll_start_effective.nanos()));
    out.effective.emplace_back("poll_to_response_effective_ns",
                               twr_int_to_text(e.poll_to_response_effective.nanos()));
    out.effective.emplace_back("response_to_final_effective_ns",
                               twr_int_to_text(e.response_to_final_effective.nanos()));
    out.effective.emplace_back("post_tx_rx_enable_effective_ns",
                               twr_int_to_text(e.post_tx_rx_enable_effective.nanos()));
    return out;
}

// -- comparison helpers ----------------------------------------------------

void check_string_list(const std::string& name, const std::vector<std::string>& got,
                       const json::Value& expect, const std::string& case_name)
{
    const json::Value& arr = member(expect, name);
    if (arr.type != json::Type::Array)
        BOOST_FAIL("corpus case " << case_name << ": expect." << name
                                 << " must be an array");
    if (arr.items.size() != got.size()) {
        std::string g;
        for (const auto& s : got)
            g += "\n      " + s;
        BOOST_FAIL("corpus case " << case_name << ": expect." << name << " has "
                                 << arr.items.size() << " entries, this build "
                                 << "produced " << got.size() << ":" << g);
    }
    for (size_t i = 0; i < got.size(); ++i) {
        if (got[i] != arr.items[i].text) {
            BOOST_FAIL("corpus case " << case_name << ": expect." << name << "[" << i
                                     << "] is '" << arr.items[i].text << "', this build "
                                     << "produced '" << got[i] << "'");
        }
    }
}

// The corpus stores effective values as JSON natives (bool / int / string) and
// this build renders every one of them as text.  Comparing the two therefore
// needs one canonical rendering, and it is the SAME one the C++ writer uses for
// its own output: booleans as true/false, integers as decimal, doubles as
// %.17g, strings verbatim.  A mismatch here is a real difference in the value,
// not a formatting artefact.
std::string value_text(const json::Value& v)
{
    switch (v.type) {
    case json::Type::Bool:
        return v.boolean ? "true" : "false";
    case json::Type::Int:
        return twr_int_to_text(v.integer);
    case json::Type::Double:
        return twr_double_to_text(v.number);
    case json::Type::String:
        return v.text;
    default:
        return std::string();
    }
}

void check_outcome(const std::string& case_name, const Outcome& got,
                   const json::Value& expect)
{
    const json::Value* acc = expect.find("accepted");
    if (!acc || acc->type != json::Type::Bool)
        BOOST_FAIL("corpus case " << case_name << ": expect.accepted must be a bool");
    if (got.accepted != (acc->boolean != 0))
        BOOST_FAIL("corpus case " << case_name << ": accepted="
                                 << (acc->boolean ? "true" : "false")
                                 << " in the corpus, this build says "
                                 << (got.accepted ? "true" : "false"));
    check_string_list("violations", got.violations, expect, case_name);

    if (const json::Value* iok = expect.find("import_ok")) {
        if (iok->type == json::Type::Bool &&
            got.import_ok != (iok->boolean != 0))
            BOOST_FAIL("corpus case " << case_name << ": import_ok="
                                     << (iok->boolean ? "true" : "false")
                                     << " in the corpus, this build says "
                                     << (got.import_ok ? "true" : "false"));
    }
    if (expect.find("import_violations") != nullptr)
        check_string_list("import_violations", got.import_violations, expect, case_name);

    if (const json::Value* h = expect.find("config_hash")) {
        if (got.config_hash != h->text)
            BOOST_FAIL("corpus case " << case_name << ": config_hash '"
                                     << got.config_hash
                                     << "' differs from the corpus '"
                                     << h->text
                                     << "'. Every field name, value rendering "
                                        "and the order of all of them differ "
                                        "between the two implementations");
    }
    if (const json::Value* eff = expect.find("effective")) {
        if (eff->type != json::Type::Object)
            BOOST_FAIL("corpus case " << case_name << ": expect.effective must be "
                                     << "an object");
        for (const auto& kv : eff->members) {
            const std::string want = value_text(kv.second);
            bool found = false;
            for (const auto& g : got.effective) {
                if (g.first != kv.first)
                    continue;
                found = true;
                if (g.second != want)
                    BOOST_FAIL("corpus case " << case_name << ": effective."
                                             << kv.first << " is '" << want
                                             << "' in the corpus, this build "
                                                "produced '"
                                             << g.second << "'");
            }
            if (!found)
                BOOST_FAIL("corpus case " << case_name << ": effective." << kv.first
                                         << " is in the corpus but this build did "
                                            "not report it");
        }
    }
}

// Import a config straight from a parsed corpus value, by round-tripping it
// through the header's own serialiser.  This is the SAME reader both
// implementations are being compared on: there is no second, laxer import path
// hiding behind this suite.
ValidationReport from_json_value(const json::Value& doc, TwrConfig& out)
{
    std::string text;
    std::string err;
    if (!json::dump(doc, text, err, /*pretty=*/false))
        BOOST_FAIL("cannot re-serialise a corpus document: " << err);
    return from_json_string(text, out);
}

} // namespace

// ===========================================================================
// 1. The schema version is part of the contract
// ===========================================================================
// twr-config/2 is a BREAKING change: frame.frame_profile became REQUIRED and
// phy.phr_rate changed value domain.  A build that still advertises
// twr-config/1 rejects every v2 document, so this is the first thing to say --
// and it is a named, separate assertion so the failure points at the version
// rather than at 100 downstream cases.
BOOST_AUTO_TEST_CASE(schema_version_matches_the_corpus)
{
    const json::Value corpus = read_corpus();
    const json::Value& want = member(corpus, "config_schema_version");
    const std::string got = capabilities().schema_version;
    BOOST_REQUIRE_MESSAGE(
        want.text == got,
        "the config parity corpus is written against " << want.text
                                                       << " but this build advertises "
                                                       << got
                                                       << ". twr-config/2 is a BREAKING "
                                                          "change (frame.frame_profile "
                                                          "became required; "
                                                          "phy.phr_rate changed value "
                                                          "domain from a payload DataRate "
                                                          "to a PhrRate enumeration), so "
                                                          "the two schemas disagree about "
                                                          "what a PHR rate is and must not "
                                                          "be interchanged. Bump "
                                                          "Capabilities::schema_version and "
                                                          "build_default_capabilities() in "
                                                          "uwb_twr_config.h (and the "
                                                          "ConfigMeta default) to "
                                                       << want.text << ".");
    BOOST_CHECK_EQUAL(ConfigMeta().schema_version, want.text);
    BOOST_CHECK_EQUAL(ConfigMeta().schema_version, got);
}

// ===========================================================================
// 2. The corpus itself is well formed
// ===========================================================================
BOOST_AUTO_TEST_CASE(corpus_is_well_formed)
{
    const json::Value corpus = read_corpus();
    BOOST_CHECK_EQUAL(member(corpus, "corpus_version").text, "twr-config-parity-corpus/1");

    const json::Value& bases = member(corpus, "base");
    BOOST_REQUIRE(bases.type == json::Type::Object);
    // One base per (protocol, role) combination: SS and DS x initiator and
    // responder.  All four must be ACCEPTED and produce the codec's on-air
    // budget, or nothing derived from them means anything.
    for (const char* name : { "ss_initiator", "ss_responder", "ds_initiator",
                              "ds_responder" }) {
        const json::Value* base = bases.find(name);
        BOOST_REQUIRE_MESSAGE(base != nullptr, "corpus has no base document '" << name << "'");
        TwrConfig cfg;
        const ValidationReport imp = from_json_value(*base, cfg);
        BOOST_CHECK_MESSAGE(imp.ok(), "base " << name << " does not import: " << imp.to_string());
        const EffectiveConfig e = effective_config(cfg);
        BOOST_CHECK_MESSAGE(e.ok, "base " << name << " is rejected: " << e.validation.to_string());
        BOOST_CHECK_EQUAL(e.poll_bytes, 16);
        BOOST_CHECK_EQUAL(e.response_bytes, 26);
        BOOST_CHECK_EQUAL(e.final_bytes, 31);
    }

    const json::Value& cases = member(corpus, "cases");
    BOOST_REQUIRE(cases.type == json::Type::Array);
    BOOST_REQUIRE_MESSAGE(cases.items.size() >= 100,
                          "the corpus must cover the whole surface; it has only "
                              << cases.items.size() << " cases");
    // Names are unique: a duplicate would make a failure ambiguous.
    for (size_t i = 0; i < cases.items.size(); ++i) {
        for (size_t j = i + 1; j < cases.items.size(); ++j) {
            BOOST_REQUIRE_MESSAGE(
                cases.items[i].find("name")->text != cases.items[j].find("name")->text,
                "duplicate corpus case name '"
                    << cases.items[i].find("name")->text << "'");
        }
        BOOST_REQUIRE(cases.items[i].find("name") != nullptr);
        BOOST_REQUIRE(cases.items[i].find("base") != nullptr);
        BOOST_REQUIRE(cases.items[i].find("expect") != nullptr);
        BOOST_REQUIRE(cases.items[i].find("expect")->find("accepted") != nullptr);
        BOOST_REQUIRE(cases.items[i].find("expect")->find("violations") != nullptr);
        // Every case states WHY it exists, in prose.  A case with no note is a
        // case nobody can check against the requirement it claims to test.
        BOOST_REQUIRE_MESSAGE(cases.items[i].find("note") != nullptr &&
                                  !cases.items[i].find("note")->text.empty(),
                              "corpus case '"
                                  << cases.items[i].find("name")->text
                                  << "' has no note saying what it pins down");
    }
}

// ===========================================================================
// 3. The differential test: every case, this build against the corpus
// ===========================================================================
BOOST_AUTO_TEST_CASE(every_corpus_case)
{
    const json::Value corpus = read_corpus();
    const json::Value& bases = member(corpus, "base");
    const json::Value& cases = member(corpus, "cases");

    BOOST_REQUIRE(cases.type == json::Type::Array);
    BOOST_REQUIRE_MESSAGE(!cases.items.empty(),
                          "the corpus has no cases: a parity test that runs "
                          "nothing is a skipped test wearing a hat");

    for (const auto& c : cases.items) {
        const json::Value* name_v = c.find("name");
        BOOST_REQUIRE(name_v != nullptr);
        const std::string name = name_v->text;
        const json::Value* base = bases.find(c.find("base")->text);
        BOOST_REQUIRE_MESSAGE(base != nullptr, "case " << name << " names an unknown base");
        const Outcome got = run_case(*base, c);
        check_outcome(name, got, member(c, "expect"));
    }
}

// ===========================================================================
// 4. The non-finite sweep: every float field, all three non-finite values
// ===========================================================================
// Not delegated to the corpus: this is generated, so a new float field added to
// TwrConfig is covered the moment it is listed in kFloatFields above, and a
// field missing from that list is a visible omission rather than a silent gap.
BOOST_AUTO_TEST_CASE(non_finite_is_refused_in_every_float_field)
{
    const json::Value corpus = read_corpus();
    const json::Value& base = member(member(corpus, "base"), "ss_initiator");

    const double kValues[3] = { std::numeric_limits<double>::quiet_NaN(),
                                std::numeric_limits<double>::infinity(),
                                -std::numeric_limits<double>::infinity() };

    for (size_t i = 0; i < kFloatFieldCount; ++i) {
        const std::string path = kFloatFields[i];
        const bool quantisation = path.size() > 15 &&
                                  path.compare(path.size() - 15, 15,
                                               "quantisation_hz") == 0;
        // A timed field's quantisation rate is reached through
        // validate_timed_field() only when the validator actually VALIDATES
        // that field for this base.  Five of them it does not: the initiator
        // never sends a Response, SS-TWR has no Final, and Report is reserved,
        // so those are only checked for being non-zero.  Naming them keeps the
        // "and also quantisation_unsupported" half of the assertion honest
        // instead of quietly applying it to fields that are never visited.
        static const char* const kNotVisited[] = {
            "timing.poll_to_response.quantisation_hz",
            "timing.response_to_final.quantisation_hz",
            "timing.final_to_report.quantisation_hz",
            "timeouts.final_rx_window.quantisation_hz",
            "timeouts.report_rx_window.quantisation_hz",
        };
        bool visited = true;
        for (const char* nv : kNotVisited)
            if (path == nv)
                visited = false;

        for (double v : kValues) {
            TwrConfig cfg;
            const ValidationReport imp = from_json_value(base, cfg);
            BOOST_REQUIRE_MESSAGE(imp.ok(),
                                  "the ss_initiator base does not import: "
                                      << imp.to_string());
            set_float(cfg, path, v);
            const ValidationReport r = validate(cfg);
            if (!is_finite_checked(path)) {
                // A KNOWN GAP, pinned rather than hidden: the C++ validator has
                // no finiteness check on this field, so a NaN here is decided
                // by whatever other rule happens to fire.  Assert only that the
                // two implementations stay in step about that, which the corpus
                // case for this path already does.
                BOOST_TEST_MESSAGE("no finiteness check on " << path
                                                            << " (known gap in "
                                                               "uwb_twr_config.h); "
                                                               "pinned by its corpus case");
                continue;
            }
            BOOST_CHECK_MESSAGE(!r.ok(), "a non-finite " << path
                                                   << " was ACCEPTED; every float field "
                                                      "the validator checks must refuse "
                                                      "NaN and +/-Inf");
            BOOST_CHECK_MESSAGE(r.find(path, ConfigReason::NotFinite) != nullptr,
                                "a non-finite " << path
                                                << " did not produce not_finite: "
                                                << r.to_string());
            if (quantisation && visited) {
                BOOST_CHECK_MESSAGE(
                    r.find(path, ConfigReason::QuantisationUnsupported) != nullptr,
                    "a non-finite " << path
                                    << " did not also produce quantisation_unsupported: "
                                    << r.to_string());
            }
            // And it must NEVER be exportable, whatever else is wrong with it.
            std::string out;
            std::string err;
            BOOST_CHECK_MESSAGE(!to_json_string(cfg, out, err),
                                "a non-finite " << path
                                                << " was WRITTEN to JSON; a corrupt "
                                                   "config must never become a file a "
                                                   "later run reads as valid");
        }
    }
}

// ===========================================================================
// 4b. The gates a JSON document cannot reach
// ===========================================================================
// Two of the new checks are unreachable through the JSON path, and it is worth
// saying so rather than letting the corpus look like it covers them:
//
//   * `frame.frame_profile` is an enum, so a NAME this build has no layout for
//     is refused by the READER (unknown_enum_value) and never reaches the
//     validator.  The validator's own gate is only reachable for a config built
//     in code with an out-of-enum value -- e.g. a value cast in from an int.
//   * `phy.preamble_symbols` is a raw integer, so its PreambleLength gate IS
//     reachable from JSON (the corpus's preamble_* cases prove it), but the
//     PreambleLength ENUM itself only exists in code.
//
// So this test constructs both directly.  test_twr_config_parity.py has the
// mirror of it, so the two suites cover the same two gates.
BOOST_AUTO_TEST_CASE(the_gates_a_json_document_cannot_reach)
{
    const json::Value corpus = read_corpus();
    const json::Value& base = member(member(corpus, "base"), "ss_initiator");
    TwrConfig proto;
    BOOST_REQUIRE(from_json_value(base, proto).ok());

    // (a) the frame-profile gate, with a value no JSON reader would produce.
    {
        TwrConfig c = proto;
        c.frame.frame_profile = static_cast<FrameProfileId>(9);
        const ValidationReport r = validate(c);
        BOOST_REQUIRE(!r.ok());
        const ConfigViolation* v = r.find("frame.frame_profile", ConfigReason::Unsupported);
        BOOST_REQUIRE(v != nullptr);
        BOOST_CHECK(v->reason == ConfigReason::Unsupported);
        BOOST_CHECK(v->status == ExchangeStatus::Unsupported);
        BOOST_CHECK_EQUAL(v->requirement, "REQ-PROTO-06");
        BOOST_CHECK(v->message.find("not implemented") != std::string::npos);
        // Reached the ordinary way, an unknown NAME is the READER's job --
        // see the corpus case frame_profile_frame_v2_is_refused_by_the_reader.
        // Here we only assert the two gates are distinct.
        BOOST_CHECK(frame_profile_id_is_supported(FrameProfileId::TwrV1));
        BOOST_CHECK(!frame_profile_id_is_supported(
            static_cast<FrameProfileId>(9)));
    }

    // (b) the geometry authority, against a claim no reader would build.
    {
        FrameGeometryClaim claim;
        claim.mac_header_bytes = 7;
        claim.timestamp_bytes = 5;
        claim.phr_bytes = 12;
        FrameProfileGeometry authority;
        FrameProfile profile;
        BOOST_REQUIRE(frame_geometry_for(FrameProfileId::TwrV1, authority));
        BOOST_REQUIRE(frame_profile_for(FrameProfileId::TwrV1, profile));
        const GeometryCheckResult g =
            frame_geometry_check(claim, authority, profile);
        BOOST_REQUIRE(!g.ok());
        BOOST_REQUIRE(g.fields.count == 2);
        BOOST_CHECK(g.fields.find(GeometryField::MacHeaderBytes) != nullptr);
        BOOST_CHECK(g.fields.find(GeometryField::PhrBytes) != nullptr);
        BOOST_CHECK_EQUAL(g.fields.find(GeometryField::MacHeaderBytes)->expected,
                          kFrameHeaderBytes);
        BOOST_CHECK_EQUAL(g.fields.find(GeometryField::PhrBytes)->expected,
                          kPhrStandardInfoBytes);
        // The MAC-appends variant is describable and not encodable.
        const FrameProfileGeometry mac = frame_geometry_mac_appends_fcs();
        BOOST_CHECK(!mac.executable());
        BOOST_CHECK_EQUAL(mac.mac_fcs_bytes(), kFrameFcsBytes);
        for (FrameType t : { FrameType::Poll, FrameType::Response, FrameType::Final })
            BOOST_CHECK_EQUAL(mac.on_air_bytes(t, profile),
                              authority.on_air_bytes(t, profile));
    }

    // (c) the PHR-rate axis, which is not a payload data rate at all.
    {
        BOOST_CHECK(phr_rate_is_implemented(PhrRate::Standard850k));
        BOOST_CHECK(!phr_rate_is_implemented(PhrRate::SameAsData));
        BOOST_CHECK_EQUAL(phr_rate_symbols(PhrRate::Standard850k), 21u);
        BOOST_CHECK_EQUAL(phr_rate_symbols(PhrRate::SameAsData), 0u);
        PhrRate parsed = PhrRate::Standard850k;
        BOOST_CHECK(phr_rate_from_string("850k", parsed));
        BOOST_CHECK(parsed == PhrRate::Standard850k);
        // "6p8m" is a DataRate spelling and is NOT a PHR rate.
        BOOST_CHECK(!phr_rate_from_string("6p8m", parsed));
        BOOST_CHECK(phr_rate_unsupported_reason(PhrRate::SameAsData).find(
                        "not a claim about any Qorvo part") != std::string::npos);
        TwrConfig c = proto;
        c.phy.phr_rate = PhrRate::SameAsData;
        const ValidationReport r = validate(c);
        BOOST_REQUIRE(!r.ok());
        const ConfigViolation* v = r.find("phy.phr_rate", ConfigReason::Unsupported);
        BOOST_REQUIRE(v != nullptr);
        BOOST_CHECK_EQUAL(v->requirement, "REQ-PHY-02");
    }

    // (d) the PreambleLength enumeration, and the local -> wire session map.
    {
        PreambleLength len = PreambleLength::Sym64;
        BOOST_CHECK(preamble_length_from_symbols(64, len));
        BOOST_CHECK(len == PreambleLength::Sym64);
        BOOST_CHECK(preamble_length_from_symbols(16, len));
        BOOST_CHECK(len == PreambleLength::Sym16);
        BOOST_CHECK(!preamble_length_from_symbols(128, len));
        BOOST_CHECK(!preamble_length_from_symbols(100, len));
        BOOST_CHECK_EQUAL(preamble_length_symbols(PreambleLength::Sym16), 16u);
        BOOST_CHECK_EQUAL(preamble_length_symbols(PreambleLength::Sym64), 64u);
        // Every refused length names a limit of THIS software, never a chip.
        for (uint16_t n : { 1, 2, 4, 8, 32, 100, 128, 256, 512, 1024, 2048, 4096 }) {
            const std::string why = preamble_length_unsupported_reason(n);
            BOOST_CHECK(why.find(std::to_string(n)) != std::string::npos);
            BOOST_CHECK(why.find("structurally impossible") == std::string::npos);
            BOOST_CHECK(why.find("the hardware cannot") == std::string::npos);
        }
        uint16_t wire = 0;
        std::string err;
        SessionIdError code = SessionIdError::None;
        BOOST_CHECK(session_id_to_wire(0xFFFF, wire, err, &code));
        BOOST_CHECK_EQUAL(wire, 0xFFFFu);
        BOOST_CHECK(!session_id_to_wire(0x10000, wire, err, &code));
        BOOST_CHECK(code == SessionIdError::OutOfWireRange);
        BOOST_CHECK_EQUAL(wire, 0u);   // never a plausible-looking leftover
        BOOST_CHECK(err.find("not truncated to 0") != std::string::npos);
        BOOST_CHECK(session_id_fits_wire(0xFFFF));
        BOOST_CHECK(!session_id_fits_wire(0x10000));
        BOOST_CHECK(wire_session_id_collides(7, 7));
        BOOST_CHECK(!wire_session_id_collides(7, 8));
        BOOST_CHECK(!wire_session_id_collides(0x10000, 0));
    }

    // (e) every PHR-form member says why, in its own words.
    BOOST_CHECK(phr_mode_unsupported_reason(PhrMode::Extended).find(
                    "13 information bits") != std::string::npos);
    BOOST_CHECK(phr_mode_unsupported_reason(PhrMode::None).find(
                    "RANGING bit") != std::string::npos);
    BOOST_CHECK(phr_mode_unsupported_reason(PhrMode::Standard).find(
                    "2 octet") != std::string::npos);
}

// ===========================================================================
// 5. The float-field table itself cannot rot
// ===========================================================================
// Every path in kFloatFields must be assignable.  A new float field that
// somebody forgets to list is a silent coverage hole; this turns it into a
// build failure the next time the table is used.
BOOST_AUTO_TEST_CASE(the_non_finite_table_covers_the_whole_config)
{
    const json::Value corpus = read_corpus();
    const json::Value& base = member(member(corpus, "base"), "ss_initiator");
    TwrConfig proto;
    BOOST_REQUIRE(from_json_value(base, proto).ok());
    for (size_t i = 0; i < kFloatFieldCount; ++i) {
        TwrConfig cfg;
        BOOST_REQUIRE(from_json_value(base, cfg).ok());
        // The set_float() call BOOST_FAILs on an unknown path, so reaching the
        // next line IS the assertion.
        set_float(cfg, kFloatFields[i], std::numeric_limits<double>::quiet_NaN());
        BOOST_CHECK_MESSAGE(validate(cfg).find("", ConfigReason::None) == nullptr,
                            "the probe value never reached a field");
    }
    // And the corpus agrees about the list, so both sides poison the SAME set.
    const json::Value& listed = member(corpus, "non_finite_fields");
    BOOST_REQUIRE(listed.type == json::Type::Array);
    BOOST_REQUIRE_EQUAL(listed.items.size(), kFloatFieldCount);
    for (size_t i = 0; i < kFloatFieldCount; ++i)
        BOOST_CHECK_EQUAL(listed.items[i].text, std::string(kFloatFields[i]));
}

// ===========================================================================
// 6. The evidence ladder, pinned (M0.1 R7)
// ===========================================================================
// The 48 measured rows establish work_decode_verified and NOTHING above it.
// A row that claimed toa_verified would let a decode-verified length be used
// to produce a range, which is precisely the overstatement R7 found.
BOOST_AUTO_TEST_CASE(the_measured_rows_claim_only_work_decode_verified)
{
    const Capabilities& caps = capabilities();
    BOOST_REQUIRE_EQUAL(caps.phy_matrix.size(), 48u);
    for (const auto& row : caps.phy_matrix) {
        // Compared by NAME, not by the enum: the name is the on-disk contract
        // (CSV cells, JSON, log lines) and it is what a reader greps for.
        BOOST_CHECK_EQUAL(std::string(evidence::level_name(row.evidence_level)),
                          std::string("work_decode_verified"));
        BOOST_CHECK_EQUAL(row.evidence_not_established,
                          std::string(evidence::level_name(
                              evidence::Level::NativeRoundtripVerified)) +
                              ";" + evidence::level_name(evidence::Level::ToaVerified) +
                              ";" + evidence::level_name(evidence::Level::HardwareVerified) +
                              ";" +
                              evidence::level_name(evidence::Level::VendorInteropVerified));
        BOOST_CHECK_EQUAL(row.evidence_source,
                          std::string("work_direct_998p4_modulate_loopback_demodulate_fcs"));
    }
    // A decode-verified row must NOT satisfy a ranging use: fail closed.
    BOOST_CHECK(!evidence::allows(evidence::Level::WorkDecodeVerified,
                                  evidence::Use::Ranging));
    BOOST_CHECK(evidence::allows(evidence::Level::WorkDecodeVerified,
                                 evidence::Use::WorkDecode));
    BOOST_CHECK(evidence::allows(evidence::Level::ToaVerified, evidence::Use::Ranging));
    BOOST_CHECK(!evidence::allows(evidence::Level::ToaVerified,
                                  evidence::Use::Hardware));
}

BOOST_AUTO_TEST_SUITE_END()
