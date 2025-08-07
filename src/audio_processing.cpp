#include "audio_processing.h"
#include "webrtc/modules/audio_processing/include/audio_processing.h"
#include "webrtc/modules/audio_processing/audio_buffer.h"
#include "webrtc/modules/audio_processing/echo_cancellation_impl.h"
#include "webrtc/modules/audio_processing/aec/aec_core_internal.h"
#include "webrtc/common_audio/channel_buffer.h"
#include "webrtc/base/checks.h"
#include "webrtc/common_audio/include/audio_util.h"

#include <iostream>
#include <stdexcept>
#include <cstring>
#include <algorithm>

class AudioProcessing::AudioProcessingPrivate {
public:
    AudioProcessingPrivate();
    ~AudioProcessingPrivate();

    int sample_rate_;
    int system_delay_ms_;
    int noise_suppression_level_;
    int aec_level_;
    int voice_detection_level_;
    bool enable_aec_;
    bool enable_agc_;
    bool enable_hp_filter_;
    bool enable_noise_suppression_;
    bool enable_transient_suppression_;
    bool aec_delay_agnostic_;
    bool aec_extended_filter_;
    bool enable_voice_detection_;
    int agc_mode_;

    std::unique_ptr<webrtc::AudioProcessing> audio_processor_;
    std::unique_ptr<webrtc::StreamConfig> stream_config_in_;
    std::unique_ptr<webrtc::StreamConfig> stream_config_out_;

    std::vector<float> near_float_data_;
    std::vector<float> far_float_data_;
    std::vector<float> out_float_data_;
    std::unique_ptr<webrtc::ChannelBuffer<float>> near_chan_buf_;
    std::unique_ptr<webrtc::ChannelBuffer<float>> far_chan_buf_;
    std::unique_ptr<webrtc::ChannelBuffer<float>> out_chan_buf_;

    size_t num_chunk_samples_;
    bool is_started_;

    void configureProcessing();
    void validateInputSizes(const std::vector<int16_t>& near_in,
                            const std::vector<int16_t>& far_in) const;
};

AudioProcessing::AudioProcessingPrivate::AudioProcessingPrivate()
    : sample_rate_(16000)
    , system_delay_ms_(0)
    , noise_suppression_level_(NS_LEVEL_MODERATE)
    , aec_level_(2)
    , voice_detection_level_(kModerateLikelihood)
    , enable_aec_(true)
    , enable_agc_(false)
    , enable_hp_filter_(true)
    , enable_noise_suppression_(true)
    , enable_transient_suppression_(false)
    , aec_delay_agnostic_(false)
    , aec_extended_filter_(false)
    , enable_voice_detection_(false)
    , agc_mode_(AGC_MODE_ADAPTIVE_DIGITAL)
    , num_chunk_samples_(0)
    , is_started_(false) {}

AudioProcessing::AudioProcessingPrivate::~AudioProcessingPrivate() = default;

void AudioProcessing::AudioProcessingPrivate::configureProcessing() {
    if (!audio_processor_) return;

    // AEC
    if (enable_aec_) {
        audio_processor_->echo_cancellation()->Enable(true);
        audio_processor_->echo_cancellation()->enable_drift_compensation(false);
        audio_processor_->echo_cancellation()->set_suppression_level(
            static_cast<webrtc::EchoCancellation::SuppressionLevel>(aec_level_));
        if (aec_delay_agnostic_) {
            audio_processor_->echo_cancellation()->enable_delay_logging(true);
        }
    } else {
        audio_processor_->echo_cancellation()->Enable(false);
    }

    // AGC
    if (enable_agc_) {
        audio_processor_->gain_control()->Enable(true);
        webrtc::GainControl::Mode mode = static_cast<webrtc::GainControl::Mode>(agc_mode_);
        audio_processor_->gain_control()->set_mode(mode);
    } else {
        audio_processor_->gain_control()->Enable(false);
    }

    // NS
    if (enable_noise_suppression_) {
        audio_processor_->noise_suppression()->Enable(true);
        webrtc::NoiseSuppression::Level level =
            static_cast<webrtc::NoiseSuppression::Level>(noise_suppression_level_);
        audio_processor_->noise_suppression()->set_level(level);
    } else {
        audio_processor_->noise_suppression()->Enable(false);
    }

    // HPF
    audio_processor_->high_pass_filter()->Enable(enable_hp_filter_);

    // Voice Detection
    if (enable_voice_detection_) {
        audio_processor_->voice_detection()->Enable(true);
        audio_processor_->voice_detection()->set_likelihood(
            static_cast<webrtc::VoiceDetection::Likelihood>(voice_detection_level_));
    } else {
        audio_processor_->voice_detection()->Enable(false);
    }

    if (system_delay_ms_ > 0) {
        audio_processor_->set_stream_delay_ms(system_delay_ms_);
    }
}

void AudioProcessing::AudioProcessingPrivate::validateInputSizes(
    const std::vector<int16_t>& near_in,
    const std::vector<int16_t>& far_in) const {
    if (near_in.size() != far_in.size()) {
        throw std::invalid_argument("Near and far input sizes must match");
    }
    if (near_in.empty()) {
        throw std::invalid_argument("Input vectors cannot be empty");
    }
}

AudioProcessing::AudioProcessing() : pImpl(std::make_unique<AudioProcessingPrivate>()) {}
AudioProcessing::~AudioProcessing() = default;

bool AudioProcessing::setConfig(int configId, std::variant<int, bool, float> value) {
    switch (configId) {
        case SAMPLE_RATE: pImpl->sample_rate_ = std::get<int>(value); break;
        case SYSTEM_DELAY_MS: pImpl->system_delay_ms_ = std::get<int>(value); break;
        case NOISE_SUPPRESSION_LEVEL: pImpl->noise_suppression_level_ = std::get<int>(value); break;
        case AEC_LEVEL: pImpl->aec_level_ = std::get<int>(value); break;
        case ENABLE_AEC: pImpl->enable_aec_ = std::get<bool>(value); break;
        case ENABLE_AGC: pImpl->enable_agc_ = std::get<bool>(value); break;
        case ENABLE_HP_FILTER: pImpl->enable_hp_filter_ = std::get<bool>(value); break;
        case ENABLE_NOISE_SUPPRESSION: pImpl->enable_noise_suppression_ = std::get<bool>(value); break;
        case ENABLE_TRANSIENT_SUPPRESSION: pImpl->enable_transient_suppression_ = std::get<bool>(value); break;
        case AEC_DELAY_AGNOSTIC: pImpl->aec_delay_agnostic_ = std::get<bool>(value); break;
        case AEC_EXTENDED_FILTER: pImpl->aec_extended_filter_ = std::get<bool>(value); break;
        case ENABLE_VOICE_DETECTION: pImpl->enable_voice_detection_ = std::get<bool>(value); break;
        case AGC_MODE: pImpl->agc_mode_ = std::get<int>(value); break;
        default: return false;
    }
    return true;
}

void AudioProcessing::start() {
    if (pImpl->is_started_) return;

    pImpl->audio_processor_ = std::unique_ptr<webrtc::AudioProcessing>(
        webrtc::AudioProcessing::Create());
    if (!pImpl->audio_processor_) return;

    pImpl->configureProcessing();
    pImpl->num_chunk_samples_ = pImpl->sample_rate_ / 100;

    pImpl->stream_config_in_ = std::make_unique<webrtc::StreamConfig>(
        pImpl->sample_rate_, WEBRTC_AEC3_NUM_CHANNELS);
    pImpl->stream_config_out_ = std::make_unique<webrtc::StreamConfig>(
        pImpl->sample_rate_, WEBRTC_AEC3_NUM_CHANNELS);

    pImpl->near_float_data_.resize(pImpl->num_chunk_samples_);
    pImpl->far_float_data_.resize(pImpl->num_chunk_samples_);
    pImpl->out_float_data_.resize(pImpl->num_chunk_samples_);

    pImpl->near_chan_buf_ = std::make_unique<webrtc::ChannelBuffer<float>>(
        pImpl->num_chunk_samples_, WEBRTC_AEC3_NUM_CHANNELS, pImpl->num_chunk_samples_);
    pImpl->far_chan_buf_ = std::make_unique<webrtc::ChannelBuffer<float>>(
        pImpl->num_chunk_samples_, WEBRTC_AEC3_NUM_CHANNELS, pImpl->num_chunk_samples_);
    pImpl->out_chan_buf_ = std::make_unique<webrtc::ChannelBuffer<float>>(
        pImpl->num_chunk_samples_, WEBRTC_AEC3_NUM_CHANNELS, pImpl->num_chunk_samples_);

    pImpl->is_started_ = true;
}

bool AudioProcessing::Process(const uint8_t* nearBytes, size_t nearByteCount,
                              const uint8_t* farBytes, size_t farByteCount,
                              std::vector<int16_t>& out) {
    if (!nearBytes || !farBytes || nearByteCount == 0 || farByteCount == 0 ||
        nearByteCount % 2 != 0 || farByteCount % 2 != 0) {
        return false;
    }

    size_t sampleCount = std::min(nearByteCount, farByteCount) / 2;

    std::vector<int16_t> nearSamples(sampleCount);
    std::vector<int16_t> farSamples(sampleCount);

    for (size_t i = 0; i < sampleCount; ++i) {
        nearSamples[i] = static_cast<int16_t>(nearBytes[i * 2] | (nearBytes[i * 2 + 1] << 8));
        farSamples[i] = static_cast<int16_t>(farBytes[i * 2] | (farBytes[i * 2 + 1] << 8));
    }

    if (!pImpl->is_started_ || !pImpl->audio_processor_) return false;
    pImpl->validateInputSizes(nearSamples, farSamples);

    out.resize(sampleCount);
    size_t processed = 0;

    while (processed < sampleCount) {
        size_t chunk = std::min(pImpl->num_chunk_samples_, sampleCount - processed);

        for (size_t i = 0; i < chunk; ++i) {
            pImpl->near_float_data_[i] = nearSamples[processed + i] / 32768.0f;
            pImpl->far_float_data_[i] = farSamples[processed + i] / 32768.0f;
        }

        std::copy(pImpl->near_float_data_.begin(), pImpl->near_float_data_.begin() + chunk,
                  pImpl->near_chan_buf_->channels()[0]);
        std::copy(pImpl->far_float_data_.begin(), pImpl->far_float_data_.begin() + chunk,
                  pImpl->far_chan_buf_->channels()[0]);

        if (pImpl->audio_processor_->ProcessReverseStream(pImpl->far_chan_buf_->channels(),
                                                          *pImpl->stream_config_in_,
                                                          *pImpl->stream_config_out_,
                                                          pImpl->far_chan_buf_->channels()) != 0)
            continue;

        if (pImpl->audio_processor_->ProcessStream(pImpl->near_chan_buf_->channels(),
                                                   *pImpl->stream_config_in_,
                                                   *pImpl->stream_config_out_,
                                                   pImpl->out_chan_buf_->channels()) != 0)
            continue;

        for (size_t i = 0; i < chunk; ++i) {
            float s = std::clamp(pImpl->out_chan_buf_->channels()[0][i], -1.0f, 1.0f);
            out[processed + i] = static_cast<int16_t>(s * 32767.0f);
        }

        processed += chunk;
    }

    return true;
}

bool AudioProcessing::hasVoice() const {
    return pImpl->audio_processor_ && pImpl->enable_voice_detection_ &&
           pImpl->audio_processor_->voice_detection()->stream_has_voice();
}

bool AudioProcessing::hasEcho() const {
    return pImpl->audio_processor_ && pImpl->enable_aec_ &&
           pImpl->audio_processor_->echo_cancellation()->is_enabled();
}

float AudioProcessing::getSpeechProbability() const {
    return hasVoice() ? 1.0f : 0.0f;
}

