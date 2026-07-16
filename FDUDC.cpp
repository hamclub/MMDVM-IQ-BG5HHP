/*
 * Digital up and down conversion with fractional sample rate conversion
 *
 * Implemented in a way similar to
 * https://github.com/tejeez/spektri/blob/ff7d9deaea917483afb4c151f0b3f3b10e00ffac/tools/ddc.py
 *
 */

#include "FDUDC.h"
#include <cassert>

#ifndef M_PIf32
#define M_PIf32 3.141592653589793f
#endif

static float sinc(float v)
{
    if (v == 0.0f)
        return 1.0f;
    else
        return std::sin(v) / v;
}

static float hann_window(size_t i, size_t length)
{
    return 0.5f - 0.5f * std::cos((0.5f + (float)i) / (float)length * (M_PIf32 * 2.0f));
}

static float windowed_sinc(size_t i, size_t length, float cutoff)
{
    return sinc(cutoff * ((float)i - 0.5f*float(length))) * hann_window(i, length);
}

static void make_sine_table(std::vector<std::complex<float>> &table, int freqNum, unsigned freqDen)
{
    // Frequency in radians per sample
    float freq = (float)freqNum / float(freqDen) * (M_PIf32 * 2.0f);
    table.resize(freqDen);
    for (size_t i = 0; i < (size_t)freqDen; i++) {
        table[i] = std::polar(1.0f, freq * (float)i);
    }
}

CFDUDC::CFDUDC(
    unsigned resampNum,
    unsigned resampDen,
    int      rxIfNum,
    unsigned rxIfDen,
    int      txIfNum,
    unsigned txIfDen,
    unsigned length,
    float cutoff
):
m_resampNum(resampNum),
m_resampDen(resampDen),
m_p(0),
m_i(0),
m_ddc_i(0),
m_duc_i(0),
m_ducIn(0.0f, 0.0f)
{
    size_t approxlen = (size_t)resampDen * (size_t)length;
    // Number of filter branches:
    size_t branches = resampNum;
    // resampNum determines the number of polyphase branches.
    // Ensure the length is a multiple of that, so that each
    // branch has the same number of taps.
    // Length of one branch:
    size_t branchlen = (approxlen + branches/2) / branches;
    // Total length of filter prototype:
    size_t totallen = branchlen * branches;

    m_taps.resize(totallen);
    // Cutoff frequency in radians per sample
    float sinc_cutoff = (cutoff * M_PIf32) / (float)(resampDen);
    float sum = 0.0f;
    for (size_t i = 0; i < totallen; i++) {
        float v = windowed_sinc(i, totallen, sinc_cutoff);
        m_taps[i] = v;
        sum += v;
    }

    // Scale so that the sum over one filter branch becomes 1.
    // This gives the filter unity gain at DC.
    float scaling = (float)branches / sum;
    for (size_t i = 0; i < totallen; i++) {
        m_taps[i] *= scaling;
    }

    m_in.resize(branchlen * 2);
    m_out.resize(branchlen * 2);

    make_sine_table(m_ddc_sine, -rxIfNum, rxIfDen);
    make_sine_table(m_duc_sine, txIfNum, txIfDen);
};

CFDUDC::~CFDUDC()
{
}

void CFDUDC::process(
    std::vector<std::complex<float>> &buffer,
    std::function<std::complex<float>(std::complex<float>)> process_sample
)
{
    const size_t branchlen = m_in.size() / 2;
    // Scaling needed so that DUC has unity gain in passband
    const float duc_scaling = (float)m_resampDen / (float)m_resampNum;

    for (auto &sample : buffer) {
        m_in[m_i] = m_in[m_i + branchlen] = sample * m_ddc_sine[m_ddc_i];
        if (++m_ddc_i >= m_ddc_sine.size())
            m_ddc_i = 0;

        m_p += m_resampNum;
        while (m_p >= m_resampDen) {
            m_p -= m_resampDen;
            //assert(m_p < m_resampNum);

            size_t p = (size_t)m_p;
            // Windows to sample buffers
            auto windowIn  = &m_in [m_i + 1];
            auto windowOut = &m_out[m_i + 1];
            std::complex<float> ddcOut = { 0.0f, 0.0f };
            // Do both resamplers in the same loop to make it a bit faster.
            // This adds one sample of extra delay to DUC input.
            for (size_t i = 0; i < branchlen; i++) {
                //assert(p < m_taps.size());
                auto tap = m_taps[p];
                ddcOut += windowIn[i] * tap;
                windowOut[i] += m_ducIn * tap;
                p += (size_t)m_resampNum;
            }

            m_ducIn = duc_scaling * process_sample(ddcOut);
        }

        sample = (m_out[m_i] + m_out[m_i + branchlen]) * m_duc_sine[m_duc_i];
        m_out[m_i] = { 0.0f, 0.0f };
        m_out[m_i + branchlen] = { 0.0f, 0.0f };
        if (++m_duc_i >= m_duc_sine.size())
            m_duc_i = 0;
        if (++m_i >= branchlen)
            m_i = 0;
    }
}
