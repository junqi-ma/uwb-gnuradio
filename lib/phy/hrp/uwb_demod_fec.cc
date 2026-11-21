// SPDX-License-Identifier: GPL-3.0-or-later


// Implementations for the BPRF/CL-3 FEC and RS(63,55) stages.
// The header (demod_fec.h) keeps only declarations and the inline kernels.
#include <openuwb/phy/hrp/rx/demod_fec.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <utility>
#include <vector>


namespace openuwb::demod::core::detail {
void vitdec_cl3(const int8_t* rx, size_t N, std::vector<int8_t>& out)
{
    const int INF = 1000000;
    int surf[4] = { 0, INF, INF, INF };
    Cl3BackBuffer storage;
    int8_t* back = storage.get(N);
    for (size_t t = 0; t < N; ++t) {
        const int r0 = rx[2 * t];
        const int r1 = rx[2 * t + 1];
        int8_t* bt = back + t * 4;
        int ns[4] = { INF, INF, INF, INF };
        cl3_acs_update(surf[0] + ((r0 ^ 0) + (r1 ^ 0)), 0, 0, bt + 0, ns);
        cl3_acs_update(surf[1] + ((r0 ^ 0) + (r1 ^ 1)), 0, 1, bt + 0, ns);
        cl3_acs_update(surf[2] + ((r0 ^ 1) + (r1 ^ 0)), 1, 2, bt + 1, ns);
        cl3_acs_update(surf[3] + ((r0 ^ 1) + (r1 ^ 1)), 1, 3, bt + 1, ns);
        cl3_acs_update(surf[0] + ((r0 ^ 0) + (r1 ^ 1)), 2, 4, bt + 2, ns);
        cl3_acs_update(surf[1] + ((r0 ^ 0) + (r1 ^ 0)), 2, 5, bt + 2, ns);
        cl3_acs_update(surf[2] + ((r0 ^ 1) + (r1 ^ 1)), 3, 6, bt + 3, ns);
        cl3_acs_update(surf[3] + ((r0 ^ 1) + (r1 ^ 0)), 3, 7, bt + 3, ns);
        surf[0] = ns[0];
        surf[1] = ns[1];
        surf[2] = ns[2];
        surf[3] = ns[3];
    }
    int best = 0;
    for (int i = 1; i < 4; ++i)
        if (surf[i] < surf[best])
            best = i;
    out.resize(N);
    int cur = best;
    for (size_t t = N; t-- > 0;) {
        const int v = back[t * 4 + cur];
        out[t] = static_cast<int8_t>(v >> 2);
        cur = v & 3;
    }
}

bool bprf_demod_impl(const std::vector<float>& soft, int64_t start,
                     size_t code_index, size_t nsym, size_t cpb, size_t cps,
                     size_t scram_offset, int8_t pol, std::vector<int8_t>& g0,
                     std::vector<int8_t>& g1, std::vector<float>* llr_g0,
                     std::vector<float>* llr_g1)
{
    if (start < 0 ||
        static_cast<size_t>(start + static_cast<int64_t>(cps * nsym)) >
            soft.size())
        return false;
    std::vector<int8_t> spread;
    if (!bprf_spreading(spread, code_index, scram_offset, cpb * nsym))
        return false;
    g0.resize(nsym);
    g1.resize(nsym);
    if (llr_g0) {
        llr_g0->resize(nsym);
        llr_g1->resize(nsym);
    }
    const size_t half = cps / 2;
    for (size_t s = 0; s < nsym; ++s) {
        const size_t base = static_cast<size_t>(start) + s * cps;
        const int8_t* sp = &spread[s * cpb];
        float metrics[4] = {};
        for (size_t hop = 0; hop < 2; ++hop) {
            const size_t offset = hop * cpb;
            float metric0 = 0.0f, metric1 = 0.0f;
            for (size_t c = 0; c < cpb; ++c) {
                metric0 += pol * soft[base + offset + c] * static_cast<float>(sp[c]);
                metric1 += pol * soft[base + half + offset + c] *
                           static_cast<float>(sp[c]);
            }
            metrics[2 * hop] = metric0;
            metrics[2 * hop + 1] = metric1;
        }
        const auto decision = rxopt::decide_bpm_symbol(metrics, true);
        g0[s] = decision.position;
        g1[s] = decision.polarity;
        if (llr_g0) {
            (*llr_g0)[s] = decision.position_llr;
            (*llr_g1)[s] = rxopt::bpm_polarity_llr(metrics);
        }
    }
    return true;
}

bool bprf_demod(const std::vector<float>& soft, int64_t start,
                size_t code_index, size_t nsym, size_t cpb, size_t cps,
                size_t scram_offset, int8_t pol, std::vector<int8_t>& g0,
                std::vector<int8_t>& g1)
{
    return bprf_demod_impl(soft, start, code_index, nsym, cpb, cps,
                           scram_offset, pol, g0, g1, nullptr, nullptr);
}

// Positive LLRs favour bit 0 for BPM position and BPSK polarity.
bool bprf_demod_soft(const std::vector<float>& soft, int64_t start,
                     size_t code_index, size_t nsym, size_t cpb, size_t cps,
                     size_t scram_offset, int8_t pol,
                     std::vector<int8_t>& g0, std::vector<int8_t>& g1,
                     std::vector<float>& llr_g0, std::vector<float>& llr_g1)
{
    return bprf_demod_impl(soft, start, code_index, nsym, cpb, cps,
                           scram_offset, pol, g0, g1, &llr_g0, &llr_g1);
}

void vitdec_cl3_soft(const float* llr0, const float* llr1, size_t N,
                     std::vector<int8_t>& out)
{
    const float INF = 1e30f;
    float surf[4] = { 0.0f, INF, INF, INF };
    Cl3BackBuffer storage;
    int8_t* back = storage.get(N);
    for (size_t t = 0; t < N; ++t) {
        const float l0 = llr0[t];
        const float l1 = llr1[t];
        int8_t* bt = back + t * 4;
        float ns[4] = { INF, INF, INF, INF };
        // Eight explicit edges; per target state the two predecessors keep the
        // reference's ascending `st` order (see cl3_acs_update above).
        cl3_acs_update_soft(cl3_soft_path_metric(surf[0], 0, 0, l0, l1), 0, 0, bt + 0, ns);
        cl3_acs_update_soft(cl3_soft_path_metric(surf[1], 0, 1, l0, l1), 0, 1, bt + 0, ns);
        cl3_acs_update_soft(cl3_soft_path_metric(surf[2], 1, 0, l0, l1), 1, 2, bt + 1, ns);
        cl3_acs_update_soft(cl3_soft_path_metric(surf[3], 1, 1, l0, l1), 1, 3, bt + 1, ns);
        cl3_acs_update_soft(cl3_soft_path_metric(surf[0], 0, 1, l0, l1), 2, 4, bt + 2, ns);
        cl3_acs_update_soft(cl3_soft_path_metric(surf[1], 0, 0, l0, l1), 2, 5, bt + 2, ns);
        cl3_acs_update_soft(cl3_soft_path_metric(surf[2], 1, 1, l0, l1), 3, 6, bt + 3, ns);
        cl3_acs_update_soft(cl3_soft_path_metric(surf[3], 1, 0, l0, l1), 3, 7, bt + 3, ns);
        for (int i = 0; i < 4; ++i)
            surf[i] = ns[i];
    }
    int best = 0;
    for (int i = 1; i < 4; ++i)
        if (surf[i] < surf[best])
            best = i;
    out.resize(N);
    int cur = best;
    for (size_t t = N; t-- > 0;) {
        const int v = back[t * 4 + cur];
        out[t] = static_cast<int8_t>(v >> 2);
        cur = v & 3;
    }
}

void rs_syndromes(const int* r63, int* S8)
{
    for (int i = 1; i <= 8; ++i) {
        const int ai = rs_gf_alpha_pow(i);
        int s = 0;
        for (int j = 62; j >= 0; --j)
            s = rs_gf_mul(s, ai) ^ (r63[j] & 0x3f);
        S8[i - 1] = s;
    }
}

int rs_berlekamp_massey_len(const int* S, int count, int* Lambda /* len >= 9 */)
{
    int C[9] = { 1, 0, 0, 0, 0, 0, 0, 0, 0 };
    int B[9] = { 1, 0, 0, 0, 0, 0, 0, 0, 0 };
    int L = 0;
    int m = 1;
    int b = 1;
    for (int n = 0; n < count; ++n) {
        int delta = S[n];
        for (int i = 1; i <= L; ++i)
            delta ^= rs_gf_mul(C[i], S[n - i]);
        if (delta == 0) {
            ++m;
        } else {
            int T[9];
            for (int i = 0; i < 9; ++i)
                T[i] = C[i];
            const int scale = rs_gf_mul(delta, rs_gf_inv(b));
            for (int i = 0; i < 9 - m; ++i)
                C[i + m] ^= rs_gf_mul(scale, B[i]);
            if (2 * L <= n) {
                L = n + 1 - L;
                for (int i = 0; i < 9; ++i)
                    B[i] = T[i];
                b = delta;
                m = 1;
            } else {
                ++m;
            }
        }
    }
    for (int i = 0; i < 9; ++i)
        Lambda[i] = C[i];
    return L;
}

int rs_berlekamp_massey(const int* S8, int* Lambda /* len >= 9 */)
{
    return rs_berlekamp_massey_len(S8, 8, Lambda);
}

int rs_chien_search(const int* Lambda, int L, int* err_pos /* <=4 */)
{
    int nerr = 0;
    for (int j = 0; j < 63; ++j) {
        const int xinv = rs_gf_alpha_inv_pow(j); // alpha^{-j}
        int v = Lambda[L] & 0x3f;
        for (int i = L - 1; i >= 0; --i)
            v = rs_gf_mul(v, xinv) ^ (Lambda[i] & 0x3f);
        if (v == 0) {
            if (nerr < 4)
                err_pos[nerr] = j;
            ++nerr;
        }
    }
    return nerr;
}

void rs_forney(const int* S8, const int* Lambda, int L,
               const int* err_pos, int nerr, int* err_val)
{
    int Omega[8] = {};
    for (int i = 0; i < 8; ++i) {
        int o = 0;
        for (int j = 0; j <= L && j <= i; ++j)
            o ^= rs_gf_mul(Lambda[j], S8[i - j]);
        Omega[i] = o;
    }
    for (int k = 0; k < nerr; ++k) {
        const int j = err_pos[k];
        const int Xinv = rs_gf_alpha_inv_pow(j);
        // num = Omega(X^{-1}) by Horner over Omega[0..7].
        int num = Omega[7] & 0x3f;
        for (int i = 6; i >= 0; --i)
            num = rs_gf_mul(num, Xinv) ^ (Omega[i] & 0x3f);
        // Lambda' in char 2: only odd powers, Lambda_i * x^{i-1}
        // = sum_k Lambda_{1+2k} (X^{-2})^k, evaluated by Horner in X^{-2}.
        const int x2 = rs_gf_mul(Xinv, Xinv);
        const int last_odd = (L & 1) ? L : L - 1;
        int den = 0;
        for (int i = last_odd; i >= 1; i -= 2)
            den = rs_gf_mul(den, x2) ^ (Lambda[i] & 0x3f);
        err_val[k] = den ? rs_gf_mul(num, rs_gf_inv(den)) : 0;
    }
}

bool rs_apply_corrections(int* symbols, const int* syndromes,
                          const int* locator, int degree,
                          const int* positions, int count)
{
    int corrections[8] = {};
    rs_forney(syndromes, locator, degree, positions, count, corrections);
    for (int i = 0; i < count; ++i)
        symbols[positions[i]] ^= corrections[i];
    int check[8];
    rs_syndromes(symbols, check);
    return rs_zero_syndromes(check);
}

void expand_rs_codeword(const int8_t* coded, size_t data_bits, int8_t* block)
{
    const size_t pad = 330 - data_bits;
    std::fill_n(block, pad, int8_t{0});
    for (size_t i = 0; i < data_bits + 48; ++i)
        block[pad + i] = static_cast<int8_t>(coded[i] & 1);
}

bool rs_decode_block(const int8_t* coded378, int8_t* data330)
{
    int r[63];
    rs_bits_to_syms(coded378, r);

    int S[8];
    rs_syndromes(r, S);
    if (!rs_zero_syndromes(S)) {
        int Lambda[9] = {};
        const int L = rs_berlekamp_massey(S, Lambda);
        if (L <= 0 || L > 4)
            return false;
        int err_pos[4] = {};
        const int nerr = rs_chien_search(Lambda, L, err_pos);
        if (nerr != L)
            return false;
        if (!rs_apply_corrections(r, S, Lambda, L, err_pos, nerr))
            return false;
    }

    rs_syms_to_bits(r, 55, data330);
    return true;
}

bool rs_decode_stream(const std::vector<int8_t>& coded,
                      size_t data_bits,
                      std::vector<int8_t>& data_out)
{
    data_out.clear();
    data_out.reserve(data_bits);
    if (data_bits == 0)
        return true;

    const size_t num_blocks = (data_bits + 329) / 330;
    size_t coded_off = 0;
    size_t data_off = 0;
    for (size_t b = 0; b < num_blocks; ++b) {
        const size_t this_data =
            std::min<size_t>(330, data_bits - data_off);
        const size_t this_coded = this_data + 48;
        if (coded_off + this_coded > coded.size())
            return false;

        int8_t block378[378];
        expand_rs_codeword(coded.data() + coded_off, this_data, block378);
        const size_t data_start = 330 - this_data;
        int8_t data330[330];
        if (!rs_decode_block(block378, data330))
            return false;

        for (size_t i = 0; i < this_data; ++i)
            data_out.push_back(data330[data_start + i]);

        coded_off += this_coded;
        data_off += this_data;
    }
    return data_out.size() == data_bits;
}

void pack_psdu_bytes(const std::vector<int8_t>& bits, std::vector<uint8_t>& bytes)
{
    bytes.resize(bits.size() / 8);
    for (size_t i = 0; i < bytes.size(); ++i) {
        const int8_t* b = bits.data() + 8 * i;
        // LSB-first packing: byte bit k = bits[8*i + k] (unchanged semantics,
        // including the uint8_t truncation of each shifted term).
        const uint8_t byte =
            static_cast<uint8_t>(static_cast<uint8_t>(b[0]) |
                                 (static_cast<uint8_t>(b[1]) << 1) |
                                 (static_cast<uint8_t>(b[2]) << 2) |
                                 (static_cast<uint8_t>(b[3]) << 3) |
                                 (static_cast<uint8_t>(b[4]) << 4) |
                                 (static_cast<uint8_t>(b[5]) << 5) |
                                 (static_cast<uint8_t>(b[6]) << 6) |
                                 (static_cast<uint8_t>(b[7]) << 7));
        bytes[i] = byte;
    }
}

bool check_psdu_fcs(const std::vector<uint8_t>& bytes,
                    uint16_t& calculated, uint16_t& received)
{
    if (bytes.size() < 2)
        return false;
    const size_t fcs_offset = bytes.size() - 2;
    calculated = crc16_802154(bytes.data(), fcs_offset);
    received = static_cast<uint16_t>(bytes[fcs_offset]) |
               (static_cast<uint16_t>(bytes[fcs_offset + 1]) << 8);
    return received == calculated;
}

bool finish_rs_crc_payload(const std::vector<int8_t>& rs_cw,
                           size_t psdu_bits,
                           PayloadResult& out,
                           PayloadSubTimers* sub,
                           PayloadDecodeWorkspace* ws)
{
    out = PayloadResult{};
    std::vector<int8_t> local_psdu_bits;
    std::vector<int8_t>& psdu_bits_out =
        ws ? ws->psdu_bits : local_psdu_bits;
    const auto t_rs = std::chrono::steady_clock::now();
    const bool rs_ok = rs_decode_stream(rs_cw, psdu_bits, psdu_bits_out);
    if (sub)
        sub->rs_us += payload_elapsed_us(t_rs);
    if (!rs_ok)
        return false;

    // PSDU assembly + FCS: bit packing, CRC-16 and the comparison.
    const auto t_crc = std::chrono::steady_clock::now();
    out.bits.assign(psdu_bits_out.begin(), psdu_bits_out.end());
    pack_psdu_bytes(psdu_bits_out, out.bytes);
    out.fcs_pass = check_psdu_fcs(out.bytes, out.calculated_fcs, out.received_fcs);
    if (sub)
        sub->crc_us += payload_elapsed_us(t_crc);
    out.ok = true;
    return true;
}

bool decode_payload_hard(const std::vector<int8_t>& g0p,
                         const std::vector<int8_t>& g1p,
                         const std::vector<int8_t>& g0,
                         const std::vector<int8_t>& g1,
                         size_t psdu_bits,
                         PayloadResult& out,
                         PayloadSubTimers* sub,
                         PayloadDecodeWorkspace* ws)
{
    PayloadSubTimer sub_guard(sub);
    out = PayloadResult{};
    if (g0p.size() != 21 || g1p.size() != 21 || g0.empty() ||
        g0.size() != g1.size())
        return false;

    const size_t total = 21 + g0.size();
    std::vector<int8_t> local_rx_bits, local_decoded, local_rs_cw;
    std::vector<int8_t>& rx_bits = ws ? ws->rx_bits : local_rx_bits;
    std::vector<int8_t>& decoded = ws ? ws->decoded : local_decoded;
    std::vector<int8_t>& rs_cw = ws ? ws->rs_cw : local_rs_cw;
    rx_bits.resize(2 * total);
    for (size_t s = 0; s < 21; ++s) {
        rx_bits[2 * s] = g0p[s];
        rx_bits[2 * s + 1] = g1p[s];
    }
    for (size_t s = 0; s < g0.size(); ++s) {
        rx_bits[2 * (s + 21)] = g0[s];
        rx_bits[2 * (s + 21) + 1] = g1[s];
    }
    const auto t_vit = std::chrono::steady_clock::now();
    vitdec_cl3(rx_bits.data(), total, decoded);
    if (sub)
        sub->viterbi_us += payload_elapsed_us(t_vit);
    if (decoded.size() < total)
        return false;

    rs_cw.assign(decoded.begin() + 19, decoded.begin() + (total - 2));
    if (rs_cw.size() != g0.size())
        return false;
    return finish_rs_crc_payload(rs_cw, psdu_bits, out, sub, ws);
}

void vitdec_cl3_soft_trace(const float* llr0,
                           const float* llr1,
                           size_t N,
                           Cl3TrellisTrace& tr)
{
    const float INF = 1e30f;
    float surf[4] = { 0.0f, INF, INF, INF };
    std::vector<int8_t> back(N * 4);
    std::vector<float> diff(N * 4, INF);
    std::vector<uint8_t> alt(N * 4, 0);
    for (size_t t = 0; t < N; ++t) {
        const float l0 = llr0[t];
        const float l1 = llr1[t];
        int8_t* bt = back.data() + t * 4;
        float ns[4] = { INF, INF, INF, INF };
        float ns2[4] = { INF, INF, INF, INF };
        uint8_t nalt[4] = { 0, 0, 0, 0 };
        // Same eight edges in the same (st, u) global order as the reference
        // nested loop; per target state the two predecessors stay in
        // ascending st order, so the runner-up (`ns2`/`nalt`) selection and
        // every strict-comparison tie break are unchanged.
        cl3_trace_update(cl3_soft_path_metric(surf[0], 0, 0, l0, l1), 0, 0, bt + 0, ns, ns2, nalt);
        cl3_trace_update(cl3_soft_path_metric(surf[0], 0, 1, l0, l1), 2, 4, bt + 2, ns, ns2, nalt);
        cl3_trace_update(cl3_soft_path_metric(surf[1], 0, 1, l0, l1), 0, 1, bt + 0, ns, ns2, nalt);
        cl3_trace_update(cl3_soft_path_metric(surf[1], 0, 0, l0, l1), 2, 5, bt + 2, ns, ns2, nalt);
        cl3_trace_update(cl3_soft_path_metric(surf[2], 1, 0, l0, l1), 1, 2, bt + 1, ns, ns2, nalt);
        cl3_trace_update(cl3_soft_path_metric(surf[2], 1, 1, l0, l1), 3, 6, bt + 3, ns, ns2, nalt);
        cl3_trace_update(cl3_soft_path_metric(surf[3], 1, 1, l0, l1), 1, 3, bt + 1, ns, ns2, nalt);
        cl3_trace_update(cl3_soft_path_metric(surf[3], 1, 0, l0, l1), 3, 7, bt + 3, ns, ns2, nalt);
        for (int i = 0; i < 4; ++i) {
            surf[i] = ns[i];
            diff[t * 4 + i] = ns2[i] - ns[i];
            alt[t * 4 + i] = nalt[i];
        }
    }
    int best = 0;
    for (int i = 1; i < 4; ++i)
        if (surf[i] < surf[best])
            best = i;
    tr.bits.resize(N);
    tr.states.resize(N + 1);
    tr.margins.resize(N);
    tr.lose_u.resize(N);
    int cur = best;
    tr.states[N] = static_cast<int8_t>(cur);
    for (size_t t = N; t-- > 0;) {
        const int v = back[t * 4 + cur];
        tr.bits[t] = static_cast<int8_t>(v >> 2);
        tr.margins[t] = diff[t * 4 + cur];
        tr.lose_u[t] = static_cast<uint8_t>(alt[t * 4 + cur] >> 2);
        cur = v & 3;
        tr.states[t] = static_cast<int8_t>(cur);
    }
}

void local_cl3_trellis_flip(const float* llr0,
                            const float* llr1,
                            size_t lo,
                            size_t hi,
                            int start_st,
                            int end_st,
                            const size_t* force_t,
                            const uint8_t* force_u,
                            size_t n_force,
                            std::vector<int8_t>& out_bits)
{
    int o0[4][2], o1[4][2], nxt[4][2];
    cl3_trellis_tables(o0, o1, nxt);
    const size_t len = hi - lo;
    const float INF = 1e30f;
    float surf[4] = { INF, INF, INF, INF };
    surf[start_st] = 0.0f;
    std::vector<int8_t> back(len * 4, 0);
    for (size_t i = 0; i < len; ++i) {
        const size_t t = lo + i;
        int forced = -1;
        for (size_t f = 0; f < n_force; ++f) {
            if (force_t[f] == t) {
                forced = static_cast<int>(force_u[f]);
                break;
            }
        }
        float ns[4] = { INF, INF, INF, INF };
        for (int st = 0; st < 4; ++st) {
            if (surf[st] > 1e20f)
                continue;
            for (int u = 0; u < 2; ++u) {
                if (forced >= 0 && u != forced)
                    continue;
                const float m = cl3_soft_path_metric(
                    surf[st], o0[st][u], o1[st][u], llr0[t], llr1[t]);
                const int nst = nxt[st][u];
                if (m < ns[nst]) {
                    ns[nst] = m;
                    back[i * 4 + nst] = static_cast<int8_t>((u << 2) | st);
                }
            }
        }
        for (int s = 0; s < 4; ++s)
            surf[s] = ns[s];
    }
    int cur = end_st;
    if (surf[cur] > 1e20f) {
        for (int s = 0; s < 4; ++s)
            if (surf[s] < surf[cur])
                cur = s;
    }
    for (size_t i = len; i-- > 0;) {
        const int v = back[i * 4 + cur];
        out_bits[lo + i] = static_cast<int8_t>(v >> 2);
        cur = v & 3;
    }
}

bool rs_decode_block_erasures(const int8_t* coded378,
                              const int* eras_pos,
                              int v,
                              int8_t* data330)
{
    if (v <= 0 || v > 6)
        return rs_decode_block(coded378, data330);
    int r[63];
    rs_bits_to_syms(coded378, r);
    int S[8];
    rs_syndromes(r, S);
    if (rs_zero_syndromes(S)) {
        rs_syms_to_bits(r, 55, data330);
        return true;
    }

    int Gamma[9] = { 1, 0, 0, 0, 0, 0, 0, 0, 0 };
    for (int m = 0; m < v; ++m) {
        const int a = rs_gf_pow(2, eras_pos[m]);
        for (int d = m + 1; d >= 1; --d)
            Gamma[d] ^= rs_gf_mul(Gamma[d - 1], a);
    }
    const int rem = 8 - v;
    int T[8] = {};
    for (int k = 0; k < rem; ++k) {
        int val = 0;
        for (int j = 0; j <= v; ++j)
            val ^= rs_gf_mul(Gamma[j], S[v + k - j]);
        T[k] = val;
    }
    int C[9] = {};
    const int Le = rs_berlekamp_massey_len(T, rem, C);
    if (2 * Le + v > 8)
        return false;
    int err_pos[4] = {};
    int nerr = 0;
    if (Le > 0) {
        nerr = rs_chien_search(C, Le, err_pos);
        if (nerr != Le)
            return false;
    }
    int Psi[9] = {};
    for (int i = 0; i <= v; ++i)
        for (int j = 0; j <= Le; ++j)
            if (i + j <= 8)
                Psi[i + j] ^= rs_gf_mul(Gamma[i], C[j]);
    int all_pos[8] = {};
    int n_tot = 0;
    for (int i = 0; i < v; ++i)
        all_pos[n_tot++] = eras_pos[i];
    for (int i = 0; i < nerr; ++i) {
        for (int j = 0; j < v; ++j)
            if (err_pos[i] == eras_pos[j])
                return false;
        all_pos[n_tot++] = err_pos[i];
    }
    if (!rs_apply_corrections(r, S, Psi, Le + v, all_pos, n_tot))
        return false;
    rs_syms_to_bits(r, 55, data330);
    return true;
}

bool rescue_payload_soft(const float* llr0,
                         const float* llr1,
                         size_t total,
                         size_t psdu_bits,
                         const Cl3TrellisTrace& tr,
                         PayloadResult& out,
                         PayloadSubTimers* sub)
{
    if (psdu_bits == 0 || (psdu_bits % 8) != 0)
        return false;
    if (tr.bits.size() < total)
        return false;
    const size_t num_blocks = (psdu_bits + 329) / 330;
    struct BlockCands {
        std::vector<std::array<int8_t, 330>> cands;
    };
    std::vector<BlockCands> bc(num_blocks);
    // Per-block geometry, hoisted so both phases can extract a codeword.
    std::vector<size_t> blk_data_start(num_blocks, 0);
    std::vector<size_t> blk_coded_start(num_blocks, 0);
    std::vector<size_t> blk_this_data(num_blocks, 0);
    std::vector<size_t> blk_this_coded(num_blocks, 0);
    std::vector<uint8_t> need_flips(num_blocks, 0);
    {
        size_t coded_off = 0;
        size_t data_off = 0;
        for (size_t b = 0; b < num_blocks; ++b) {
            const size_t this_data = std::min<size_t>(330, psdu_bits - data_off);
            const size_t this_coded = this_data + 48;
            if (19 + coded_off + this_coded > total)
                return false;
            blk_data_start[b] = 330 - this_data;
            blk_coded_start[b] = 19 + coded_off;
            blk_this_data[b] = this_data;
            blk_this_coded[b] = this_coded;
            coded_off += this_coded;
            data_off += this_data;
        }
    }

    auto extract_block = [&](size_t b, const std::vector<int8_t>& dec,
                             int8_t* block378) {
        expand_rs_codeword(dec.data() + blk_coded_start[b], blk_this_data[b], block378);
    };
    auto try_append = [&](size_t b, const std::array<int8_t, 330>& cand) {
        for (const auto& ex : bc[b].cands)
            if (ex == cand)
                return;
        bc[b].cands.push_back(cand);
    };

    // ---- Phase 1: RS only (no Trellis-Chase) ------------------------------
    for (size_t b = 0; b < num_blocks; ++b) {
        int8_t blk[378];
        std::array<int8_t, 330> d330{};
        extract_block(b, tr.bits, blk);
        if (rs_decode_block(blk, d330.data())) {
            bc[b].cands.push_back(d330);
            continue;
        }
        need_flips[b] = 1;
        // Rank the RS symbols (6 bits each) by their least-reliable bit and
        // erase the worst 2/4/6.  `data_start`-relative bits that fall in the
        // 48 parity bits map to the parity region of the codeword.
        const size_t data_start = blk_data_start[b];
        const size_t this_data = blk_this_data[b];
        const size_t t_b = blk_coded_start[b];
        struct SymRel {
            int sym;
            float min_m;
        };
        std::vector<SymRel> srel;
        const int first_sym = static_cast<int>(data_start / 6);
        for (int sy = first_sym; sy < 63; ++sy) {
            float mm = 1e30f;
            for (int bit = 0; bit < 6; ++bit) {
                const size_t pos = static_cast<size_t>(sy * 6 + bit);
                size_t vt = 0;
                if (pos >= data_start && pos < 330)
                    vt = t_b + (pos - data_start);
                else if (pos >= 330)
                    vt = t_b + this_data + (pos - 330);
                else
                    continue;
                mm = std::min(mm, tr.margins[vt]);
            }
            srel.push_back({ sy, mm });
        }
        std::sort(srel.begin(), srel.end(),
                  [](const SymRel& a, const SymRel& c) {
                      return a.min_m < c.min_m;
                  });
        for (int v : { 2, 4, 6 }) {
            if (static_cast<int>(srel.size()) < v)
                break;
            int ep[6];
            for (int k = 0; k < v; ++k)
                ep[k] = srel[k].sym;
            if (rs_decode_block_erasures(blk, ep, v, d330.data())) {
                try_append(b, d330);
                break;
            }
        }
    }

    std::vector<size_t> idx(num_blocks, 0);
    std::vector<int8_t> psdu_out(psdu_bits);
    std::vector<uint8_t> bytes(psdu_bits / 8);
    // Combined-candidate CRC search.  Returns true and fills `out` on a match.
    auto combine_and_crc = [&](size_t max_iter) -> bool {
        std::fill(idx.begin(), idx.end(), 0);
        size_t iter = 0;
        while (iter < max_iter) {
            size_t doff = 0;
            for (size_t b = 0; b < num_blocks; ++b) {
                const size_t this_data = blk_this_data[b];
                const size_t data_start = blk_data_start[b];
                const auto& c = bc[b].cands[idx[b]];
                for (size_t i = 0; i < this_data; ++i)
                    psdu_out[doff + i] = c[data_start + i];
                doff += this_data;
            }
            pack_psdu_bytes(psdu_out, bytes);
            uint16_t calculated = 0, received = 0;
            if (check_psdu_fcs(bytes, calculated, received)) {
                out = PayloadResult{};
                out.ok = true;
                out.fcs_pass = true;
                out.bytes = bytes;
                out.bits.assign(psdu_out.begin(), psdu_out.end());
                out.calculated_fcs = calculated;
                out.received_fcs = received;
                return true;
            }
            ++iter;
            size_t pos = num_blocks;
            while (pos-- > 0) {
                if (++idx[pos] < bc[pos].cands.size())
                    break;
                idx[pos] = 0;
            }
            if (pos == static_cast<size_t>(-1))
                break;
        }
        return false;
    };

    bool all_have_cands = true;
    for (size_t b = 0; b < num_blocks; ++b)
        if (bc[b].cands.empty())
            all_have_cands = false;
    if (all_have_cands && combine_and_crc(64))
        return true;

    // ---- Phase 2: local Trellis-Chase on the still-failing blocks ---------
    std::vector<int8_t> work_bits;
    for (size_t b = 0; b < num_blocks; ++b) {
        if (!need_flips[b])
            continue;
        int8_t blk[378];
        std::array<int8_t, 330> d330{};
        const size_t t_b = blk_coded_start[b];
        const size_t t_e = t_b + blk_this_coded[b];
        struct StepM {
            size_t t;
            float m;
        };
        std::vector<StepM> sm;
        sm.reserve(blk_this_coded[b]);
        for (size_t t = t_b; t < t_e; ++t)
            sm.push_back({ t, tr.margins[t] });
        std::sort(sm.begin(), sm.end(),
                  [](const StepM& a, const StepM& c) { return a.m < c.m; });
        std::vector<size_t> weak;
        for (const auto& s : sm) {
            bool near = false;
            for (size_t w : weak) {
                if (std::llabs(static_cast<int64_t>(s.t) -
                               static_cast<int64_t>(w)) < 3) {
                    near = true;
                    break;
                }
            }
            if (!near) {
                weak.push_back(s.t);
                if (weak.size() >= 6)
                    break;
            }
        }
        if (work_bits.empty())
            work_bits = tr.bits;
        for (size_t i = 0; i < weak.size() && bc[b].cands.size() < 4; ++i) {
            const size_t t = weak[i];
            const size_t lo = (t > 12) ? (t - 12) : 0;
            const size_t hi = std::min(total, t + 13);
            std::copy(tr.bits.begin() + lo, tr.bits.begin() + hi,
                      work_bits.begin() + lo);
            const uint8_t fu = tr.lose_u[t];
            local_cl3_trellis_flip(llr0, llr1, lo, hi, tr.states[lo],
                                   tr.states[hi], &t, &fu, 1, work_bits);
            extract_block(b, work_bits, blk);
            if (rs_decode_block(blk, d330.data()))
                try_append(b, d330);
        }
        for (size_t i = 0; i < weak.size() && bc[b].cands.size() < 4; ++i) {
            for (size_t j = i + 1;
                 j < weak.size() && bc[b].cands.size() < 4; ++j) {
                const size_t t0 = std::min(weak[i], weak[j]);
                const size_t t1 = std::max(weak[i], weak[j]);
                const size_t lo = (t0 > 12) ? (t0 - 12) : 0;
                const size_t hi = std::min(total, t1 + 13);
                std::copy(tr.bits.begin() + lo, tr.bits.begin() + hi,
                          work_bits.begin() + lo);
                const size_t ft[2] = { t0, t1 };
                const uint8_t fu[2] = { tr.lose_u[t0], tr.lose_u[t1] };
                local_cl3_trellis_flip(llr0, llr1, lo, hi, tr.states[lo],
                                       tr.states[hi], ft, fu, 2, work_bits);
                extract_block(b, work_bits, blk);
                if (rs_decode_block(blk, d330.data()))
                    try_append(b, d330);
            }
        }
        if (bc[b].cands.empty())
            return false;
    }

    if (combine_and_crc(64))
        return true;
    return false;
}

bool decode_payload_soft(const std::vector<float>& llr0p,
                         const std::vector<float>& llr1p,
                         const std::vector<float>& llr0,
                         const std::vector<float>& llr1,
                         size_t psdu_bits,
                         PayloadResult& out,
                         PayloadSubTimers* sub,
                         PayloadDecodeWorkspace* ws)
{
    PayloadSubTimer sub_guard(sub);
    out = PayloadResult{};
    if (llr0p.size() != 21 || llr1p.size() != 21 || llr0.empty() ||
        llr0.size() != llr1.size())
        return false;

    const size_t total = 21 + llr0.size();
    std::vector<float> local_llr0j, local_llr1j;
    std::vector<float>& llr0j = ws ? ws->llr0_joint : local_llr0j;
    std::vector<float>& llr1j = ws ? ws->llr1_joint : local_llr1j;
    llr0j.resize(total);
    llr1j.resize(total);
    for (size_t s = 0; s < 21; ++s) {
        llr0j[s] = llr0p[s];
        llr1j[s] = llr1p[s];
    }
    for (size_t s = 0; s < llr0.size(); ++s) {
        llr0j[s + 21] = llr0[s];
        llr1j[s + 21] = llr1[s];
    }
    Cl3TrellisTrace tr;
    const auto t_vit = std::chrono::steady_clock::now();
    // One tracing Viterbi serves both the first attempt and the rescue.  The
    // rescue needs the per-step path-metric margins, and recomputing them with
    // a second full-length pass was pure duplication: `tr.bits` is bit-identical
    // to what vitdec_cl3_soft() produces (same state/back-pointer updates).
    vitdec_cl3_soft_trace(llr0j.data(), llr1j.data(), total, tr);
    if (sub)
        sub->viterbi_us += payload_elapsed_us(t_vit);
    if (tr.bits.size() < total)
        return false;

    std::vector<int8_t> local_rs_cw;
    std::vector<int8_t>& rs_cw = ws ? ws->rs_cw : local_rs_cw;
    rs_cw.assign(tr.bits.begin() + 19, tr.bits.begin() + (total - 2));
    if (rs_cw.size() != llr0.size())
        return false;
    if (finish_rs_crc_payload(rs_cw, psdu_bits, out, sub, ws) && out.fcs_pass)
        return true;

    PayloadResult rescued;
    if (rescue_payload_soft(llr0j.data(), llr1j.data(), total, psdu_bits, tr,
                            rescued, sub)) {
        out = std::move(rescued);
        return true;
    }
    return out.ok;
}

void vitdec_cl3_biorth(const float* m0, const float* m1, size_t N,
                       std::vector<int8_t>& out,
                       bool terminated)
{
    const float INF = 1e30f;
    float surf[4] = { 0.0f, INF, INF, INF };
    Cl3BackBuffer storage;
    int8_t* back = storage.get(N);
    for (size_t t = 0; t < N; ++t) {
        const float a0 = m0[t];
        const float a1 = m1[t];
        int8_t* bt = back + t * 4;
        float ns[4] = { INF, INF, INF, INF };
        // Eight explicit edges in the same (st, u) order as vitdec_cl3_soft():
        // (0,0) (0,1) (1,0) (1,1) (2,0) (2,1) (3,0) (3,1).  Branch values are
        // spelled out from cl3_biorth_branch() so the kernel stays branch-free.
        cl3_acs_update_soft(surf[0] - a0, 0, 0, bt + 0, ns);
        cl3_acs_update_soft(surf[1] + a0, 0, 1, bt + 0, ns);
        cl3_acs_update_soft(surf[2] - a1, 1, 2, bt + 1, ns);
        cl3_acs_update_soft(surf[3] + a1, 1, 3, bt + 1, ns);
        cl3_acs_update_soft(surf[0] + a0, 2, 4, bt + 2, ns);
        cl3_acs_update_soft(surf[1] - a0, 2, 5, bt + 2, ns);
        cl3_acs_update_soft(surf[2] + a1, 3, 6, bt + 3, ns);
        cl3_acs_update_soft(surf[3] - a1, 3, 7, bt + 3, ns);
        for (int i = 0; i < 4; ++i)
            surf[i] = ns[i];
    }
    int best = 0;
    if (!terminated) {
        for (int i = 1; i < 4; ++i)
            if (surf[i] < surf[best])
                best = i;
    }
    out.resize(N);
    int cur = best;
    for (size_t t = N; t-- > 0;) {
        const int v = back[t * 4 + cur];
        out[t] = static_cast<int8_t>(v >> 2);
        cur = v & 3;
    }
}

void vitdec_cl3_biorth_trace(const float* m0, const float* m1, size_t N,
                             Cl3TrellisTrace& tr,
                             bool terminated)
{
    const float INF = 1e30f;
    float surf[4] = { 0.0f, INF, INF, INF };
    Cl3BackBuffer storage;
    int8_t* back = storage.get(N);
    std::vector<float> diff(N * 4, INF);
    std::vector<uint8_t> alt(N * 4, 0);
    for (size_t t = 0; t < N; ++t) {
        const float a0 = m0[t];
        const float a1 = m1[t];
        int8_t* bt = back + t * 4;
        float ns[4] = { INF, INF, INF, INF };
        float ns2[4] = { INF, INF, INF, INF };
        uint8_t nalt[4] = { 0, 0, 0, 0 };
        cl3_trace_update(surf[0] - a0, 0, 0, bt + 0, ns, ns2, nalt);
        cl3_trace_update(surf[0] + a0, 2, 4, bt + 2, ns, ns2, nalt);
        cl3_trace_update(surf[1] + a0, 0, 1, bt + 0, ns, ns2, nalt);
        cl3_trace_update(surf[1] - a0, 2, 5, bt + 2, ns, ns2, nalt);
        cl3_trace_update(surf[2] - a1, 1, 2, bt + 1, ns, ns2, nalt);
        cl3_trace_update(surf[2] + a1, 3, 6, bt + 3, ns, ns2, nalt);
        cl3_trace_update(surf[3] + a1, 1, 3, bt + 1, ns, ns2, nalt);
        cl3_trace_update(surf[3] - a1, 3, 7, bt + 3, ns, ns2, nalt);
        for (int i = 0; i < 4; ++i) {
            surf[i] = ns[i];
            diff[t * 4 + i] = ns2[i] - ns[i];
            alt[t * 4 + i] = nalt[i];
        }
    }
    int best = 0;
    if (!terminated) {
        for (int i = 1; i < 4; ++i)
            if (surf[i] < surf[best])
                best = i;
    }
    tr.bits.resize(N);
    tr.states.resize(N + 1);
    tr.margins.resize(N);
    tr.lose_u.resize(N);
    int cur = best;
    tr.states[N] = static_cast<int8_t>(cur);
    for (size_t t = N; t-- > 0;) {
        const int v = back[t * 4 + cur];
        tr.bits[t] = static_cast<int8_t>(v >> 2);
        tr.margins[t] = diff[t * 4 + cur];
        tr.lose_u[t] = static_cast<uint8_t>(alt[t * 4 + cur] >> 2);
        cur = v & 3;
        tr.states[t] = static_cast<int8_t>(cur);
    }
}

void local_cl3_trellis_flip_biorth(const float* m0,
                                   const float* m1,
                                   size_t lo,
                                   size_t hi,
                                   int start_st,
                                   int end_st,
                                   const size_t* force_t,
                                   const uint8_t* force_u,
                                   size_t n_force,
                                   std::vector<int8_t>& out_bits)
{
    int o0[4][2], o1[4][2], nxt[4][2];
    cl3_trellis_tables(o0, o1, nxt);
    const size_t len = hi - lo;
    const float INF = 1e30f;
    float surf[4] = { INF, INF, INF, INF };
    surf[start_st] = 0.0f;
    std::vector<int8_t> back(len * 4, 0);
    for (size_t i = 0; i < len; ++i) {
        const size_t t = lo + i;
        int forced = -1;
        for (size_t f = 0; f < n_force; ++f) {
            if (force_t[f] == t) {
                forced = static_cast<int>(force_u[f]);
                break;
            }
        }
        float ns[4] = { INF, INF, INF, INF };
        for (int st = 0; st < 4; ++st) {
            if (surf[st] > 1e20f)
                continue;
            for (int u = 0; u < 2; ++u) {
                if (forced >= 0 && u != forced)
                    continue;
                const float m = surf[st] + cl3_biorth_branch(
                    m0[t], m1[t], o0[st][u], o1[st][u]);
                const int nst = nxt[st][u];
                if (m < ns[nst]) {
                    ns[nst] = m;
                    back[i * 4 + nst] = static_cast<int8_t>((u << 2) | st);
                }
            }
        }
        for (int s = 0; s < 4; ++s)
            surf[s] = ns[s];
    }
    int cur = end_st;
    if (surf[cur] > 1e20f) {
        for (int s = 0; s < 4; ++s)
            if (surf[s] < surf[cur])
                cur = s;
    }
    for (size_t i = len; i-- > 0;) {
        const int v = back[i * 4 + cur];
        out_bits[lo + i] = static_cast<int8_t>(v >> 2);
        cur = v & 3;
    }
}

void clip_biorth_metrics(float* m0, float* m1, size_t N,
                         float clip_factor)
{
    if (!m0 || !m1 || N < 16 || !(clip_factor > 0.0f))
        return;
    constexpr size_t kMaxSamples = 1024;
    float sample[kMaxSamples];
    size_t ns = 0;
    if (N <= kMaxSamples) {
        for (size_t i = 0; i < N; ++i)
            sample[i] = std::max(std::abs(m0[i]), std::abs(m1[i]));
        ns = N;
    } else {
        const size_t stride = (N + kMaxSamples - 1) / kMaxSamples;
        for (size_t i = 0; i < N && ns < kMaxSamples; i += stride)
            sample[ns++] = std::max(std::abs(m0[i]), std::abs(m1[i]));
    }
    if (ns == 0)
        return;
    const size_t mid = ns / 2;
    std::nth_element(sample, sample + mid, sample + ns);
    const float median = sample[mid];
    if (!(median > 0.0f))
        return;
    const float limit = clip_factor * median;
    for (size_t i = 0; i < N; ++i) {
        float a = m0[i];
        if (a > limit)
            a = limit;
        else if (a < -limit)
            a = -limit;
        m0[i] = a;
        float b = m1[i];
        if (b > limit)
            b = limit;
        else if (b < -limit)
            b = -limit;
        m1[i] = b;
    }
}

bool rescue_payload_biorth(const float* m0,
                           const float* m1,
                           size_t total,
                           size_t psdu_bits,
                           const Cl3TrellisTrace& tr,
                           PayloadResult& out,
                           PayloadSubTimers* sub)
{
    (void)sub;
    if (psdu_bits == 0 || (psdu_bits % 8) != 0)
        return false;
    if (tr.bits.size() < total)
        return false;
    const size_t num_blocks = (psdu_bits + 329) / 330;
    struct BlockCands {
        std::vector<std::array<int8_t, 330>> cands;
    };
    std::vector<BlockCands> bc(num_blocks);
    std::vector<size_t> blk_data_start(num_blocks, 0);
    std::vector<size_t> blk_coded_start(num_blocks, 0);
    std::vector<size_t> blk_this_data(num_blocks, 0);
    std::vector<size_t> blk_this_coded(num_blocks, 0);
    std::vector<uint8_t> need_flips(num_blocks, 0);
    {
        size_t coded_off = 0;
        size_t data_off = 0;
        for (size_t b = 0; b < num_blocks; ++b) {
            const size_t this_data = std::min<size_t>(330, psdu_bits - data_off);
            const size_t this_coded = this_data + 48;
            if (19 + coded_off + this_coded > total)
                return false;
            blk_data_start[b] = 330 - this_data;
            blk_coded_start[b] = 19 + coded_off;
            blk_this_data[b] = this_data;
            blk_this_coded[b] = this_coded;
            coded_off += this_coded;
            data_off += this_data;
        }
    }

    auto extract_block = [&](size_t b, const std::vector<int8_t>& dec,
                             int8_t* block378) {
        expand_rs_codeword(dec.data() + blk_coded_start[b], blk_this_data[b], block378);
    };
    auto try_append = [&](size_t b, const std::array<int8_t, 330>& cand) {
        for (const auto& ex : bc[b].cands)
            if (ex == cand)
                return;
        bc[b].cands.push_back(cand);
    };

    // ---- Phase 1: RS only (no Trellis-Chase) ------------------------------
    for (size_t b = 0; b < num_blocks; ++b) {
        int8_t blk[378];
        std::array<int8_t, 330> d330{};
        extract_block(b, tr.bits, blk);
        if (rs_decode_block(blk, d330.data())) {
            bc[b].cands.push_back(d330);
            continue;
        }
        need_flips[b] = 1;
        const size_t data_start = blk_data_start[b];
        const size_t this_data = blk_this_data[b];
        const size_t t_b = blk_coded_start[b];
        struct SymRel {
            int sym;
            float min_m;
        };
        std::vector<SymRel> srel;
        const int first_sym = static_cast<int>(data_start / 6);
        for (int sy = first_sym; sy < 63; ++sy) {
            float mm = 1e30f;
            for (int bit = 0; bit < 6; ++bit) {
                const size_t pos = static_cast<size_t>(sy * 6 + bit);
                size_t vt = 0;
                if (pos >= data_start && pos < 330)
                    vt = t_b + (pos - data_start);
                else if (pos >= 330)
                    vt = t_b + this_data + (pos - 330);
                else
                    continue;
                mm = std::min(mm, tr.margins[vt]);
            }
            srel.push_back({ sy, mm });
        }
        std::sort(srel.begin(), srel.end(),
                  [](const SymRel& a, const SymRel& c) {
                      return a.min_m < c.min_m;
                  });
        for (int v : { 2, 4, 6 }) {
            if (static_cast<int>(srel.size()) < v)
                break;
            int ep[6];
            for (int k = 0; k < v; ++k)
                ep[k] = srel[k].sym;
            if (rs_decode_block_erasures(blk, ep, v, d330.data())) {
                try_append(b, d330);
                break;
            }
        }
    }

    std::vector<size_t> idx(num_blocks, 0);
    std::vector<int8_t> psdu_out(psdu_bits);
    std::vector<uint8_t> bytes(psdu_bits / 8);
    auto combine_and_crc = [&](size_t max_iter) -> bool {
        std::fill(idx.begin(), idx.end(), 0);
        size_t iter = 0;
        while (iter < max_iter) {
            size_t doff = 0;
            for (size_t b = 0; b < num_blocks; ++b) {
                const size_t this_data = blk_this_data[b];
                const size_t data_start = blk_data_start[b];
                const auto& c = bc[b].cands[idx[b]];
                for (size_t i = 0; i < this_data; ++i)
                    psdu_out[doff + i] = c[data_start + i];
                doff += this_data;
            }
            pack_psdu_bytes(psdu_out, bytes);
            uint16_t calculated = 0, received = 0;
            if (check_psdu_fcs(bytes, calculated, received)) {
                out = PayloadResult{};
                out.ok = true;
                out.fcs_pass = true;
                out.bytes = bytes;
                out.bits.assign(psdu_out.begin(), psdu_out.end());
                out.calculated_fcs = calculated;
                out.received_fcs = received;
                return true;
            }
            ++iter;
            size_t pos = num_blocks;
            while (pos-- > 0) {
                if (++idx[pos] < bc[pos].cands.size())
                    break;
                idx[pos] = 0;
            }
            if (pos == static_cast<size_t>(-1))
                break;
        }
        return false;
    };

    bool all_have_cands = true;
    for (size_t b = 0; b < num_blocks; ++b)
        if (bc[b].cands.empty())
            all_have_cands = false;
    if (all_have_cands && combine_and_crc(64))
        return true;

    // ---- Phase 2: local Trellis-Chase on the still-failing blocks ---------
    std::vector<int8_t> work_bits;
    for (size_t b = 0; b < num_blocks; ++b) {
        if (!need_flips[b])
            continue;
        int8_t blk[378];
        std::array<int8_t, 330> d330{};
        const size_t t_b = blk_coded_start[b];
        const size_t t_e = t_b + blk_this_coded[b];
        struct StepM {
            size_t t;
            float m;
        };
        std::vector<StepM> sm;
        sm.reserve(blk_this_coded[b]);
        for (size_t t = t_b; t < t_e; ++t)
            sm.push_back({ t, tr.margins[t] });
        std::sort(sm.begin(), sm.end(),
                  [](const StepM& a, const StepM& c) { return a.m < c.m; });
        std::vector<size_t> weak;
        for (const auto& s : sm) {
            bool near = false;
            for (size_t w : weak) {
                if (std::llabs(static_cast<int64_t>(s.t) -
                               static_cast<int64_t>(w)) < 3) {
                    near = true;
                    break;
                }
            }
            if (!near) {
                weak.push_back(s.t);
                if (weak.size() >= 6)
                    break;
            }
        }
        if (work_bits.empty())
            work_bits = tr.bits;
        for (size_t i = 0; i < weak.size() && bc[b].cands.size() < 4; ++i) {
            const size_t t = weak[i];
            const size_t lo = (t > 12) ? (t - 12) : 0;
            const size_t hi = std::min(total, t + 13);
            std::copy(tr.bits.begin() + lo, tr.bits.begin() + hi,
                      work_bits.begin() + lo);
            const uint8_t fu = tr.lose_u[t];
            local_cl3_trellis_flip_biorth(m0, m1, lo, hi, tr.states[lo],
                                          tr.states[hi], &t, &fu, 1, work_bits);
            extract_block(b, work_bits, blk);
            if (rs_decode_block(blk, d330.data()))
                try_append(b, d330);
        }
        for (size_t i = 0; i < weak.size() && bc[b].cands.size() < 4; ++i) {
            for (size_t j = i + 1;
                 j < weak.size() && bc[b].cands.size() < 4; ++j) {
                const size_t t0 = std::min(weak[i], weak[j]);
                const size_t t1 = std::max(weak[i], weak[j]);
                const size_t lo = (t0 > 12) ? (t0 - 12) : 0;
                const size_t hi = std::min(total, t1 + 13);
                std::copy(tr.bits.begin() + lo, tr.bits.begin() + hi,
                          work_bits.begin() + lo);
                const size_t ft[2] = { t0, t1 };
                const uint8_t fu[2] = { tr.lose_u[t0], tr.lose_u[t1] };
                local_cl3_trellis_flip_biorth(m0, m1, lo, hi, tr.states[lo],
                                              tr.states[hi], ft, fu, 2,
                                              work_bits);
                extract_block(b, work_bits, blk);
                if (rs_decode_block(blk, d330.data()))
                    try_append(b, d330);
            }
        }
        if (bc[b].cands.empty())
            return false;
    }

    if (combine_and_crc(64))
        return true;
    return false;
}

bool decode_payload_biorth_soft(const float* m0p,
                                const float* m1p,
                                const float* m0,
                                const float* m1,
                                size_t payload_symbols,
                                size_t psdu_bits,
                                PayloadResult& out,
                                PayloadSubTimers* sub,
                                PayloadDecodeWorkspace* ws,
                                bool terminated,
                                bool enable_clip)
{
    PayloadSubTimer sub_guard(sub);
    out = PayloadResult{};
    if (!m0p || !m1p || !m0 || !m1 || payload_symbols == 0)
        return false;

    const size_t total = 21 + payload_symbols;
    std::vector<float> local_m0j, local_m1j;
    std::vector<float>& m0j = ws ? ws->llr0_joint : local_m0j;
    std::vector<float>& m1j = ws ? ws->llr1_joint : local_m1j;
    m0j.resize(total);
    m1j.resize(total);
    for (size_t s = 0; s < 21; ++s) {
        m0j[s] = m0p[s];
        m1j[s] = m1p[s];
    }
    for (size_t s = 0; s < payload_symbols; ++s) {
        m0j[s + 21] = m0[s];
        m1j[s + 21] = m1[s];
    }
    if (enable_clip)
        clip_biorth_metrics(m0j.data(), m1j.data(), total);

    Cl3TrellisTrace tr;
    const auto t_vit = std::chrono::steady_clock::now();
    vitdec_cl3_biorth_trace(m0j.data(), m1j.data(), total, tr, terminated);
    if (sub)
        sub->viterbi_us += payload_elapsed_us(t_vit);
    if (tr.bits.size() < total)
        return false;

    std::vector<int8_t> local_rs_cw;
    std::vector<int8_t>& rs_cw = ws ? ws->rs_cw : local_rs_cw;
    rs_cw.assign(tr.bits.begin() + 19, tr.bits.begin() + (total - 2));
    if (rs_cw.size() != payload_symbols)
        return false;
    if (finish_rs_crc_payload(rs_cw, psdu_bits, out, sub, ws) && out.fcs_pass)
        return true;

    PayloadResult rescued;
    if (rescue_payload_biorth(m0j.data(), m1j.data(), total, psdu_bits, tr,
                              rescued, sub)) {
        out = std::move(rescued);
        return true;
    }
    return out.ok;
}

bool decode_phr_biorth_soft(const float* m0p,
                            const float* m1p,
                            const std::vector<float>& soft_chips,
                            const NsSfdResult& ns_sfd,
                            PhrResult& out)
{
    (void)soft_chips;
    (void)ns_sfd;
    out = PhrResult{};
    if (!m0p || !m1p)
        return false;

    const size_t nsym = 21;
    std::vector<int8_t> decoded;
    vitdec_cl3_biorth(m0p, m1p, nsym, decoded, false);
    if (decoded.size() < 19)
        return false;

    std::vector<int8_t> sys13(decoded.begin(), decoded.begin() + 13);
    std::vector<int8_t> recv_par(decoded.begin() + 13, decoded.begin() + 19);
    std::vector<int8_t> phr19;
    hrp_secded(sys13, phr19);
    int8_t synd[6];
    for (int i = 0; i < 6; ++i)
        synd[i] = static_cast<int8_t>(recv_par[i] ^ phr19[13 + i]);

    bool secded_pass = true;
    int idx = 0;
    for (int i = 1; i < 6; ++i)
        idx |= (synd[i] << (i - 1));
    if (idx > 0) {
        static const int kAddresses[13] = { 3, 5, 6, 7, 9, 10, 11,
                                            12, 13, 14, 15, 17, 18 };
        for (int j = 0; j < 13; ++j) {
            if (kAddresses[j] == idx) {
                phr19[j] ^= 1;
                out.secded_corrected = true;
                break;
            }
        }
        if (!synd[0])
            secded_pass = false; // DED
    }
    out.secded_uncorrectable = !secded_pass;

    // MSB-first PHR layout: bits 1..2 = data rate, bits 3..9 = PSDU length.
    uint32_t psdu = 0;
    for (int i = 0; i < 7; ++i)
        psdu |= (static_cast<uint32_t>(phr19[2 + i]) << (6 - i));
    const int dr = (phr19[0] << 1) | phr19[1];
    static const float kDataRates[4] = { 0.11f, 0.85f, 6.81f, 27.24f };
    out.data_rate_mbps = kDataRates[dr & 3];
    out.psdu_length = psdu;
    out.phr_bits.reserve(19);
    for (int i = 0; i < 19; ++i)
        out.phr_bits.push_back(static_cast<uint8_t>(phr19[i]));
    out.ok = true;
    return true;
}

} // namespace openuwb::demod::core::detail

namespace openuwb::demod::core::detail {
namespace {
// CL-3 Viterbi reference implementation (truncated mode): the double
// `for (st) for (u)` ACS loop kept as the bit-exact reference for the
// unrolled kernels below (and for cl3_trellis_tables users).  The hard and
// soft paths share path selection and traceback; metric types differ.
template <typename Metric, typename ExtendPath>
inline void vitdec_cl3_truncated(size_t N, Metric inf,
                                ExtendPath extend, std::vector<int8_t>& out)
{
    // octal 2 = 010 -> g0 = [0 1 0]; octal 5 = 101 -> g1 = [1 0 1].
    int o0[4][2], o1[4][2], nxt[4][2];
    cl3_trellis_tables(o0, o1, nxt);
    Metric surf[4] = { Metric{0}, inf, inf, inf };
    std::vector<int8_t> back(N * 4);
    for (size_t t = 0; t < N; ++t) {
        Metric ns[4] = { inf, inf, inf, inf };
        for (int st = 0; st < 4; ++st) {
            for (int u = 0; u < 2; ++u) {
                const Metric m = extend(t, surf[st], o0[st][u], o1[st][u]);
                const int nst = nxt[st][u];
                if (m < ns[nst]) {
                    ns[nst] = m;
                    back[t * 4 + nst] = static_cast<int8_t>((u << 2) | st);
                }
            }
        }
        for (int i = 0; i < 4; ++i)
            surf[i] = ns[i];
    }
    int best = 0;
    for (int i = 1; i < 4; ++i)
        if (surf[i] < surf[best])
            best = i;
    out.resize(N);
    int cur = best;
    for (size_t t = N; t-- > 0;) {
        const int v = back[t * 4 + cur];
        out[t] = static_cast<int8_t>(v >> 2);
        cur = v & 3;
    }
}
} // namespace
} // namespace openuwb::demod::core::detail
