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

class AudioProcessing::Impl {
public:
    Impl();
    ~Impl();

    // Configuration members
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

    // WebRTC objects
    std::unique_ptr<webrtc::AudioProcessing> audio_processor_;
    std::unique_ptr<webrtc::StreamConfig> stream_config_in_;
    std::unique_ptr<webrtc::StreamConfig> stream_config_out_;

    // Buffers
    std::vector<float> near_float_data_;
    std::vector<float> far_float_data_;
    std::vector<float> out_float_data_;
    std::unique_ptr<webrtc::ChannelBuffer<float>> near_chan_buf_;
    std::unique_ptr<webrtc::ChannelBuffer<float>> far_chan_buf_;
    std::unique_ptr<webrtc::ChannelBuffer<float>> out_chan_buf_;

    // Processing state
    size_t num_chunk_samples_;
    bool is_started_;

    // Methods
    void configureProcessing();
    void validateInputSizes(const std::vector<int16_t>& near_in,
                           const std::vector<int16_t>& far_in) const;
};

AudioProcessing::Impl::Impl()
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
    , is_started_(false) {
}

AudioProcessing::Impl::~Impl() {
    // Smart pointers will automatically clean up
}

void AudioProcessing::Impl::configureProcessing() {
    if (!audio_processor_) return;

    // Configure Echo Cancellation (AEC)
    if (enable_aec_) {
        audio_processor_->echo_cancellation()->Enable(true);
        audio_processor_->echo_cancellation()->enable_drift_compensation(false);
        audio_processor_->echo_cancellation()->set_suppression_level(
            static_cast<webrtc::EchoCancellation::SuppressionLevel>(aec_level_));

        // Configure delay agnostic if supported
        if (aec_delay_agnostic_) {
            audio_processor_->echo_cancellation()->enable_delay_logging(true);
        }
    } else {
        audio_processor_->echo_cancellation()->Enable(false);
    }

    // Configure AGC
    if (enable_agc_) {
        audio_processor_->gain_control()->Enable(true);
        webrtc::GainControl::Mode agc_webrtc_mode;
        switch (agc_mode_) {
            case AGC_MODE_ADAPTIVE_ANALOG:
                agc_webrtc_mode = webrtc::GainControl::kAdaptiveAnalog;
                break;
            case AGC_MODE_ADAPTIVE_DIGITAL:
                agc_webrtc_mode = webrtc::GainControl::kAdaptiveDigital;
                break;
            case AGC_MODE_FIXED_DIGITAL:
                agc_webrtc_mode = webrtc::GainControl::kFixedDigital;
                break;
            default:
                agc_webrtc_mode = webrtc::GainControl::kAdaptiveDigital;
        }
        audio_processor_->gain_control()->set_mode(agc_webrtc_mode);
    } else {
        audio_processor_->gain_control()->Enable(false);
    }

    // Configure NS
    if (enable_noise_suppression_) {
        audio_processor_->noise_suppression()->Enable(true);
        webrtc::NoiseSuppression::Level ns_level;
        switch (noise_suppression_level_) {
            case NS_LEVEL_LOW:
                ns_level = webrtc::NoiseSuppression::kLow;
                break;
            case NS_LEVEL_MODERATE:
                ns_level = webrtc::NoiseSuppression::kModerate;
                break;
            case NS_LEVEL_HIGH:
                ns_level = webrtc::NoiseSuppression::kHigh;
                break;
            case NS_LEVEL_VERY_HIGH:
                ns_level = webrtc::NoiseSuppression::kVeryHigh;
                break;
            default:
                ns_level = webrtc::NoiseSuppression::kModerate;
        }
        audio_processor_->noise_suppression()->set_level(ns_level);
    } else {
        audio_processor_->noise_suppression()->Enable(false);
    }

    // Configure HPF
    if (enable_hp_filter_) {
        audio_processor_->high_pass_filter()->Enable(true);
    } else {
        audio_processor_->high_pass_filter()->Enable(false);
    }

    // Configure Voice Detection
    if (enable_voice_detection_) {
        audio_processor_->voice_detection()->Enable(true);
        webrtc::VoiceDetection::Likelihood vad_likelihood;
        switch (voice_detection_level_) {
            case kLowLikelihood:
                vad_likelihood = webrtc::VoiceDetection::kLowLikelihood;
                break;
            case kModerateLikelihood:
                vad_likelihood = webrtc::VoiceDetection::kModerateLikelihood;
                break;
            case kHighLikelihood:
                vad_likelihood = webrtc::VoiceDetection::kHighLikelihood;
                break;
            default:
                vad_likelihood = webrtc::VoiceDetection::kModerateLikelihood;
        }
        audio_processor_->voice_detection()->set_likelihood(vad_likelihood);
    } else {
        audio_processor_->voice_detection()->Enable(false);
    }

    // Set stream delay if needed
    if (system_delay_ms_ > 0) {
        audio_processor_->set_stream_delay_ms(system_delay_ms_);
    }
}

void AudioProcessing::Impl::validateInputSizes(const std::vector<int16_t>& near_in,
                                               const std::vector<int16_t>& far_in) const {
    if (near_in.size() != far_in.size()) {
        throw std::invalid_argument("Near and far input sizes must match");
    }

    if (near_in.empty()) {
        throw std::invalid_argument("Input vectors cannot be empty");
    }
}

// AudioProcessing public interface implementation
AudioProcessing::AudioProcessing() : pImpl(std::make_unique<Impl>()) {
}

AudioProcessing::~AudioProcessing() = default;

bool AudioProcessing::setConfig(int configId, std::variant<int, bool, float> value) {
    switch (configId) {
        case SAMPLE_RATE:
            pImpl->sample_rate_ = std::get<int>(value);
            break;
        case SYSTEM_DELAY_MS:
            pImpl->system_delay_ms_ = std::get<int>(value);
            break;
        case NOISE_SUPPRESSION_LEVEL:
            pImpl->noise_suppression_level_ = std::get<int>(value);
            break;
        case AEC_LEVEL:
            pImpl->aec_level_ = std::get<int>(value);
            break;
        case ENABLE_AEC:
            pImpl->enable_aec_ = std::get<bool>(value);
            break;
        case ENABLE_AGC:
            pImpl->enable_agc_ = std::get<bool>(value);
            break;
        case ENABLE_HP_FILTER:
            pImpl->enable_hp_filter_ = std::get<bool>(value);
            break;
        case ENABLE_NOISE_SUPPRESSION:
            pImpl->enable_noise_suppression_ = std::get<bool>(value);
            break;
        case ENABLE_TRANSIENT_SUPPRESSION:
            pImpl->enable_transient_suppression_ = std::get<bool>(value);
            break;
        case AEC_DELAY_AGNOSTIC:
            pImpl->aec_delay_agnostic_ = std::get<bool>(value);
            break;
        case AEC_EXTENDED_FILTER:
            pImpl->aec_extended_filter_ = std::get<bool>(value);
            break;
        case ENABLE_VOICE_DETECTION:
            pImpl->enable_voice_detection_ = std::get<bool>(value);
            break;
        case AGC_MODE:
            pImpl->agc_mode_ = std::get<int>(value);
            break;
        default:
            return false;
    }

    return true;
}

void AudioProcessing::start() {
    if (pImpl->is_started_) return;

    pImpl->audio_processor_ = std::unique_ptr<webrtc::AudioProcessing>(
        webrtc::AudioProcessing::Create());
    if (!pImpl->audio_processor_) {
        return;
    }

    pImpl->configureProcessing();

    pImpl->num_chunk_samples_ = pImpl->sample_rate_ / 100;

    pImpl->stream_config_in_ = std::make_unique<webrtc::StreamConfig>(
        pImpl->sample_rate_, WEBRTC_AEC3_NUM_CHANNELS);
    pImpl->stream_config_out_ = std::make_unique<webrtc::StreamConfig>(
        pImpl->sample_rate_, WEBRTC_AEC3_NUM_CHANNELS);

    // Allocate float buffers
    pImpl->near_float_data_.resize(pImpl->num_chunk_samples_);
    pImpl->far_float_data_.resize(pImpl->num_chunk_samples_);
    pImpl->out_float_data_.resize(pImpl->num_chunk_samples_);

    // Create channel buffers
    pImpl->near_chan_buf_ = std::make_unique<webrtc::ChannelBuffer<float>>(
        pImpl->num_chunk_samples_, WEBRTC_AEC3_NUM_CHANNELS, pImpl->num_chunk_samples_);
    pImpl->far_chan_buf_ = std::make_unique<webrtc::ChannelBuffer<float>>(
        pImpl->num_chunk_samples_, WEBRTC_AEC3_NUM_CHANNELS, pImpl->num_chunk_samples_);
    pImpl->out_chan_buf_ = std::make_unique<webrtc::ChannelBuffer<float>>(
        pImpl->num_chunk_samples_, WEBRTC_AEC3_NUM_CHANNELS, pImpl->num_chunk_samples_);

    pImpl->is_started_ = true;
}

void AudioProcessing::Process(const std::vector<int16_t>& near_in,
                             const std::vector<int16_t>& far_in,
                             std::vector<int16_t>& out) {
    if (!pImpl->is_started_ || !pImpl->audio_processor_) {
        return;
    }

    pImpl->validateInputSizes(near_in, far_in);

    // Ensure output vector is properly sized
    out.resize(near_in.size());

    // Process audio in chunks
    size_t samples_processed = 0;
    const size_t total_samples = near_in.size();

    while (samples_processed < total_samples) {
        const size_t samples_to_process = std::min(pImpl->num_chunk_samples_,
                                                  total_samples - samples_processed);

        // Convert int16_t to float
        for (size_t i = 0; i < samples_to_process; ++i) {
            pImpl->near_float_data_[i] = static_cast<float>(near_in[samples_processed + i]) / 32768.0f;
            pImpl->far_float_data_[i] = static_cast<float>(far_in[samples_processed + i]) / 32768.0f;
        }

        // Copy to channel buffers
        std::copy(pImpl->near_float_data_.begin(),
                  pImpl->near_float_data_.begin() + samples_to_process,
                  pImpl->near_chan_buf_->channels()[0]);
        std::copy(pImpl->far_float_data_.begin(),
                  pImpl->far_float_data_.begin() + samples_to_process,
                  pImpl->far_chan_buf_->channels()[0]);

        // Process reverse stream (far-end/reference)
        int result = pImpl->audio_processor_->ProcessReverseStream(
            pImpl->far_chan_buf_->channels(), *pImpl->stream_config_in_,
            *pImpl->stream_config_out_, pImpl->far_chan_buf_->channels());

        if (result != webrtc::AudioProcessing::kNoError) {
            // Handle error
            continue;
        }

        // Process forward stream (near-end/microphone)
        result = pImpl->audio_processor_->ProcessStream(
            pImpl->near_chan_buf_->channels(), *pImpl->stream_config_in_,
            *pImpl->stream_config_out_, pImpl->out_chan_buf_->channels());

        if (result != webrtc::AudioProcessing::kNoError) {
            // Handle error
            continue;
        }

        // Convert float back to int16_t
        for (size_t i = 0; i < samples_to_process; ++i) {
            float sample = pImpl->out_chan_buf_->channels()[0][i];
            // Clamp to [-1.0, 1.0] and convert to int16_t
            sample = std::max(-1.0f, std::min(1.0f, sample));
            out[samples_processed + i] = static_cast<int16_t>(sample * 32767.0f);
        }

        samples_processed += samples_to_process;
    }
}

bool AudioProcessing::Process(const uint8_t* nearBytes, size_t nearByteCount,
                             const uint8_t* farBytes, size_t farByteCount,
                             std::vector<int16_t>& out) {
    if (!nearBytes || !farBytes || nearByteCount == 0 || farByteCount == 0) {
        return false;
    }

    // Ensure byte counts are even (since we're dealing with int16_t)
    if (nearByteCount % 2 != 0 || farByteCount % 2 != 0) {
        return false;
    }

    size_t nearSampleCount = nearByteCount / 2;
    size_t farSampleCount = farByteCount / 2;

    // Convert bytes to int16_t vectors with proper endianness handling
    std::vector<int16_t> nearSamples(nearSampleCount);
    std::vector<int16_t> farSamples(farSampleCount);

    // Assuming little-endian input (most common)
    for (size_t i = 0; i < nearSampleCount; ++i) {
        nearSamples[i] = static_cast<int16_t>(nearBytes[i * 2] | (nearBytes[i * 2 + 1] << 8));
    }

    for (size_t i = 0; i < farSampleCount; ++i) {
        farSamples[i] = static_cast<int16_t>(farBytes[i * 2] | (farBytes[i * 2 + 1] << 8));
    }

    // Ensure both vectors have the same size
    size_t minSize = std::min(nearSampleCount, farSampleCount);
    nearSamples.resize(minSize);
    farSamples.resize(minSize);

    // Process the audio using the vector overload
    Process(nearSamples, farSamples, out);
    return true;
}

bool AudioProcessing::hasVoice() const {
    if (!pImpl->audio_processor_ || !pImpl->enable_voice_detection_) {
        return false;
    }

    return pImpl->audio_processor_->voice_detection()->stream_has_voice();
}

bool AudioProcessing::hasEcho() const {
    if (!pImpl->audio_processor_ || !pImpl->enable_aec_) {
        return false;
    }

    // Check if echo cancellation is processing
    return pImpl->audio_processor_->echo_cancellation()->is_enabled();
}

float AudioProcessing::getSpeechProbability() const {
    if (!pImpl->audio_processor_ || !pImpl->enable_voice_detection_) {
        return 0.0f;
    }
    return hasVoice() ? 1.0f : 0.0f;
}











//#include "audio_processing.h"
//#include "webrtc/modules/audio_processing/include/audio_processing.h"
//#include "webrtc/modules/audio_processing/audio_buffer.h"
//#include "webrtc/modules/audio_processing/echo_cancellation_impl.h"
//#include "webrtc/modules/audio_processing/aec/aec_core_internal.h"
//#include "webrtc/common_audio/channel_buffer.h"
//#include "webrtc/base/checks.h"
//#include "webrtc/common_audio/include/audio_util.h"
//#include <iostream>
//#include <stdexcept>
//#include <cstring>
//#include <algorithm>
//#include <variant>

//class AudioProcessing::AudioProcessingPrivate {
//public:
//    AudioProcessingPrivate();
//    ~AudioProcessingPrivate();

//    // Configuration members
//    int sample_rate_;
//    int system_delay_ms_;
//    int noise_suppression_level_;
//    int aec_level_;
//    int voice_detection_level_;
//    bool enable_aec_;
//    bool enable_agc_;
//    bool enable_hp_filter_;
//    bool enable_noise_suppression_;
//    bool enable_transient_suppression_;
//    bool aec_delay_agnostic_;
//    bool aec_extended_filter_;
//    bool enable_voice_detection_;
//    int agc_mode_;

//    // WebRTC objects
//    std::unique_ptr<webrtc::AudioProcessing> audio_processor_;
//    std::unique_ptr<webrtc::StreamConfig> stream_config_in_;
//    std::unique_ptr<webrtc::StreamConfig> stream_config_out_;

//    // Buffers
//    std::vector<float> near_float_data_;
//    std::vector<float> far_float_data_;
//    std::vector<float> out_float_data_;
//    std::unique_ptr<webrtc::ChannelBuffer<float>> near_chan_buf_;
//    std::unique_ptr<webrtc::ChannelBuffer<float>> far_chan_buf_;
//    std::unique_ptr<webrtc::ChannelBuffer<float>> out_chan_buf_;

//    // Processing state
//    size_t num_chunk_samples_;
//    bool is_started_;

//    // Methods
//    void configureProcessing();
//    void validateInputSizes(const std::vector<int16_t>& near_in,
//                           const std::vector<int16_t>& far_in) const;
//};

//AudioProcessing::AudioProcessingPrivate::AudioProcessingPrivate()
//    : sample_rate_(16000)
//    , system_delay_ms_(0)
//    , noise_suppression_level_(NS_LEVEL_MODERATE)
//    , aec_level_(2)
//    , voice_detection_level_(kModerateLikelihood)
//    , enable_aec_(true)
//    , enable_agc_(false)
//    , enable_hp_filter_(true)
//    , enable_noise_suppression_(true)
//    , enable_transient_suppression_(false)
//    , aec_delay_agnostic_(false)
//    , aec_extended_filter_(false)
//    , enable_voice_detection_(false)
//    , agc_mode_(AGC_MODE_ADAPTIVE_DIGITAL)
//    , num_chunk_samples_(0)
//    , is_started_(false) {
//}

//AudioProcessing::AudioProcessingPrivate::~AudioProcessingPrivate() {
//    // Smart pointers will automatically clean up
//}

//void AudioProcessing::AudioProcessingPrivate::configureProcessing() {
//    if (!audio_processor_) return;

//    // Configure Echo Cancellation (AEC)
//    if (enable_aec_) {
//        audio_processor_->echo_cancellation()->Enable(true);
//        audio_processor_->echo_cancellation()->enable_drift_compensation(false);
//        audio_processor_->echo_cancellation()->set_suppression_level(
//            static_cast<webrtc::EchoCancellation::SuppressionLevel>(aec_level_));

//        // Configure delay agnostic if supported
//        if (aec_delay_agnostic_) {
//            audio_processor_->echo_cancellation()->enable_delay_logging(true);
//        }
//    } else {
//        audio_processor_->echo_cancellation()->Enable(false);
//    }

//    // Configure AGC
//    if (enable_agc_) {
//        audio_processor_->gain_control()->Enable(true);
//        webrtc::GainControl::Mode agc_webrtc_mode;
//        switch (agc_mode_) {
//            case AGC_MODE_ADAPTIVE_ANALOG:
//                agc_webrtc_mode = webrtc::GainControl::kAdaptiveAnalog;
//                break;
//            case AGC_MODE_ADAPTIVE_DIGITAL:
//                agc_webrtc_mode = webrtc::GainControl::kAdaptiveDigital;
//                break;
//            case AGC_MODE_FIXED_DIGITAL:
//                agc_webrtc_mode = webrtc::GainControl::kFixedDigital;
//                break;
//            default:
//                agc_webrtc_mode = webrtc::GainControl::kAdaptiveDigital;
//        }
//        audio_processor_->gain_control()->set_mode(agc_webrtc_mode);
//    } else {
//        audio_processor_->gain_control()->Enable(false);
//    }

//    // Configure NS
//    if (enable_noise_suppression_) {
//        audio_processor_->noise_suppression()->Enable(true);
//        webrtc::NoiseSuppression::Level ns_level;
//        switch (noise_suppression_level_) {
//            case NS_LEVEL_LOW:
//                ns_level = webrtc::NoiseSuppression::kLow;
//                break;
//            case NS_LEVEL_MODERATE:
//                ns_level = webrtc::NoiseSuppression::kModerate;
//                break;
//            case NS_LEVEL_HIGH:
//                ns_level = webrtc::NoiseSuppression::kHigh;
//                break;
//            case NS_LEVEL_VERY_HIGH:
//                ns_level = webrtc::NoiseSuppression::kVeryHigh;
//                break;
//            default:
//                ns_level = webrtc::NoiseSuppression::kModerate;
//        }
//        audio_processor_->noise_suppression()->set_level(ns_level);
//    } else {
//        audio_processor_->noise_suppression()->Enable(false);
//    }

//    // Configure HPF
//    if (enable_hp_filter_) {
//        audio_processor_->high_pass_filter()->Enable(true);
//    } else {
//        audio_processor_->high_pass_filter()->Enable(false);
//    }

//    // Configure Voice Detection
//    if (enable_voice_detection_) {
//        audio_processor_->voice_detection()->Enable(true);
//        webrtc::VoiceDetection::Likelihood vad_likelihood;
//        switch (voice_detection_level_) {
//            case kLowLikelihood:
//                vad_likelihood = webrtc::VoiceDetection::kLowLikelihood;
//                break;
//            case kModerateLikelihood:
//                vad_likelihood = webrtc::VoiceDetection::kModerateLikelihood;
//                break;
//            case kHighLikelihood:
//                vad_likelihood = webrtc::VoiceDetection::kHighLikelihood;
//                break;
//            default:
//                vad_likelihood = webrtc::VoiceDetection::kModerateLikelihood;
//        }
//        audio_processor_->voice_detection()->set_likelihood(vad_likelihood);
//    } else {
//        audio_processor_->voice_detection()->Enable(false);
//    }

//    // Set stream delay if needed
//    if (system_delay_ms_ > 0) {
//        audio_processor_->set_stream_delay_ms(system_delay_ms_);
//    }
//}

//void AudioProcessing::AudioProcessingPrivate::validateInputSizes(const std::vector<int16_t>& near_in,
//                                               const std::vector<int16_t>& far_in) const {
//    if (near_in.size() != far_in.size()) {
//        throw std::invalid_argument("Near and far input sizes must match");
//    }

//    if (near_in.empty()) {
//        throw std::invalid_argument("Input vectors cannot be empty");
//    }
//}

//// AudioProcessing public interface implementation
//AudioProcessing::AudioProcessing() : pImpl(std::make_unique<AudioProcessingPrivate>()) {
//}

//AudioProcessing::~AudioProcessing() = default;

//bool AudioProcessing::setConfig(int configId, std::variant<int, bool, float> value) {
//    switch (configId) {
//        case SAMPLE_RATE:
//            pImpl->sample_rate_ = std::get<int>(value);
//            break;
//        case SYSTEM_DELAY_MS:
//            pImpl->system_delay_ms_ = std::get<int>(value);
//            break;
//        case NOISE_SUPPRESSION_LEVEL:
//            pImpl->noise_suppression_level_ = std::get<int>(value);
//            break;
//        case AEC_LEVEL:
//            pImpl->aec_level_ = std::get<int>(value);
//            break;
//        case ENABLE_AEC:
//            pImpl->enable_aec_ = std::get<bool>(value);
//            break;
//        case ENABLE_AGC:
//            pImpl->enable_agc_ = std::get<bool>(value);
//            break;
//        case ENABLE_HP_FILTER:
//            pImpl->enable_hp_filter_ = std::get<bool>(value);
//            break;
//        case ENABLE_NOISE_SUPPRESSION:
//            pImpl->enable_noise_suppression_ = std::get<bool>(value);
//            break;
//        case ENABLE_TRANSIENT_SUPPRESSION:
//            pImpl->enable_transient_suppression_ = std::get<bool>(value);
//            break;
//        case AEC_DELAY_AGNOSTIC:
//            pImpl->aec_delay_agnostic_ = std::get<bool>(value);
//            break;
//        case AEC_EXTENDED_FILTER:
//            pImpl->aec_extended_filter_ = std::get<bool>(value);
//            break;
//        case ENABLE_VOICE_DETECTION:
//            pImpl->enable_voice_detection_ = std::get<bool>(value);
//            break;
//        case AGC_MODE:
//            pImpl->agc_mode_ = std::get<int>(value);
//            break;
//        default:
//            return false;
//    }

//    return true;
//}

//void AudioProcessing::start() {
//    if (pImpl->is_started_) return;

//    pImpl->audio_processor_ = std::unique_ptr<webrtc::AudioProcessing>(
//        webrtc::AudioProcessing::Create());
//    if (!pImpl->audio_processor_) {
//        return;
//    }

//    pImpl->configureProcessing();

//    pImpl->num_chunk_samples_ = pImpl->sample_rate_ / 100;

//    pImpl->stream_config_in_ = std::make_unique<webrtc::StreamConfig>(
//        pImpl->sample_rate_, WEBRTC_AEC3_NUM_CHANNELS);
//    pImpl->stream_config_out_ = std::make_unique<webrtc::StreamConfig>(
//        pImpl->sample_rate_, WEBRTC_AEC3_NUM_CHANNELS);

//    // Allocate float buffers
//    pImpl->near_float_data_.resize(pImpl->num_chunk_samples_);
//    pImpl->far_float_data_.resize(pImpl->num_chunk_samples_);
//    pImpl->out_float_data_.resize(pImpl->num_chunk_samples_);

//    // Create channel buffers
//    pImpl->near_chan_buf_ = std::make_unique<webrtc::ChannelBuffer<float>>(
//        pImpl->num_chunk_samples_, WEBRTC_AEC3_NUM_CHANNELS, pImpl->num_chunk_samples_);
//    pImpl->far_chan_buf_ = std::make_unique<webrtc::ChannelBuffer<float>>(
//        pImpl->num_chunk_samples_, WEBRTC_AEC3_NUM_CHANNELS, pImpl->num_chunk_samples_);
//    pImpl->out_chan_buf_ = std::make_unique<webrtc::ChannelBuffer<float>>(
//        pImpl->num_chunk_samples_, WEBRTC_AEC3_NUM_CHANNELS, pImpl->num_chunk_samples_);

//    pImpl->is_started_ = true;
//}

//void AudioProcessing::process(const std::vector<int16_t>& near_in,
//                             const std::vector<int16_t>& far_in,
//                             std::vector<int16_t>& out) {
//    if (!pImpl->is_started_ || !pImpl->audio_processor_) {
//        return;
//    }

//    pImpl->validateInputSizes(near_in, far_in);

//    // Ensure output vector is properly sized
//    out.resize(near_in.size());

//    // Process audio in chunks
//    size_t samples_processed = 0;
//    const size_t total_samples = near_in.size();

//    while (samples_processed < total_samples) {
//        const size_t samples_to_process = std::min(pImpl->num_chunk_samples_,
//                                                  total_samples - samples_processed);

//        // Convert int16_t to float
//        for (size_t i = 0; i < samples_to_process; ++i) {
//            pImpl->near_float_data_[i] = static_cast<float>(near_in[samples_processed + i]) / 32768.0f;
//            pImpl->far_float_data_[i] = static_cast<float>(far_in[samples_processed + i]) / 32768.0f;
//        }

//        // Copy to channel buffers
//        std::copy(pImpl->near_float_data_.begin(),
//                  pImpl->near_float_data_.begin() + samples_to_process,
//                  pImpl->near_chan_buf_->channels()[0]);
//        std::copy(pImpl->far_float_data_.begin(),
//                  pImpl->far_float_data_.begin() + samples_to_process,
//                  pImpl->far_chan_buf_->channels()[0]);

//        // Process reverse stream (far-end/reference)
//        int result = pImpl->audio_processor_->ProcessReverseStream(
//            pImpl->far_chan_buf_->channels(), *pImpl->stream_config_in_,
//            *pImpl->stream_config_out_, pImpl->far_chan_buf_->channels());

//        if (result != webrtc::AudioProcessing::kNoError) {
//            // Handle error
//            continue;
//        }

//        // Process forward stream (near-end/microphone)
//        result = pImpl->audio_processor_->ProcessStream(
//            pImpl->near_chan_buf_->channels(), *pImpl->stream_config_in_,
//            *pImpl->stream_config_out_, pImpl->out_chan_buf_->channels());

//        if (result != webrtc::AudioProcessing::kNoError) {
//            // Handle error
//            continue;
//        }

//        // Convert float back to int16_t
//        for (size_t i = 0; i < samples_to_process; ++i) {
//            float sample = pImpl->out_chan_buf_->channels()[0][i];
//            // Clamp to [-1.0, 1.0] and convert to int16_t
//            sample = std::max(-1.0f, std::min(1.0f, sample));
//            out[samples_processed + i] = static_cast<int16_t>(sample * 32767.0f);
//        }

//        samples_processed += samples_to_process;
//    }
//}

//bool AudioProcessing::processRawBytes(const uint8_t* nearBytes, size_t nearByteCount,
//                                     const uint8_t* farBytes, size_t farByteCount,
//                                     std::vector<int16_t>& out) {
//    if (!nearBytes || !farBytes || nearByteCount == 0 || farByteCount == 0) {
//        return false;
//    }

//    // Ensure byte counts are even (since we're dealing with int16_t)
//    if (nearByteCount % 2 != 0 || farByteCount % 2 != 0) {
//        return false;
//    }

//    size_t nearSampleCount = nearByteCount / 2;
//    size_t farSampleCount = farByteCount / 2;

//    // Convert bytes to int16_t vectors with proper endianness handling
//    std::vector<int16_t> nearSamples(nearSampleCount);
//    std::vector<int16_t> farSamples(farSampleCount);

//    // Assuming little-endian input (most common)
//    for (size_t i = 0; i < nearSampleCount; ++i) {
//        nearSamples[i] = static_cast<int16_t>(nearBytes[i * 2] | (nearBytes[i * 2 + 1] << 8));
//    }

//    for (size_t i = 0; i < farSampleCount; ++i) {
//        farSamples[i] = static_cast<int16_t>(farBytes[i * 2] | (farBytes[i * 2 + 1] << 8));
//    }

//    // Ensure both vectors have the same size
//    size_t minSize = std::min(nearSampleCount, farSampleCount);
//    nearSamples.resize(minSize);
//    farSamples.resize(minSize);

//    // Process the audio
//    process(nearSamples, farSamples, out);
//    return true;
//}

//bool AudioProcessing::hasVoice() const {
//    if (!pImpl->audio_processor_ || !pImpl->enable_voice_detection_) {
//        return false;
//    }

//    return pImpl->audio_processor_->voice_detection()->stream_has_voice();
//}

//bool AudioProcessing::hasEcho() const {
//    if (!pImpl->audio_processor_ || !pImpl->enable_aec_) {
//        return false;
//    }

//    // Check if echo cancellation is processing
//    return pImpl->audio_processor_->echo_cancellation()->is_enabled();
//}

//float AudioProcessing::getSpeechProbability() const {
//    if (!pImpl->audio_processor_ || !pImpl->enable_voice_detection_) {
//        return 0.0f;
//    }
//    return hasVoice() ? 1.0f : 0.0f;
//}
