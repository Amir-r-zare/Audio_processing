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
#include <variant>



AudioProcessing::AudioProcessing()
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
    , audio_processor_(nullptr)
    , stream_config_in_(nullptr)
    , stream_config_out_(nullptr)
    , near_chan_buf_(nullptr)
    , far_chan_buf_(nullptr)
    , out_chan_buf_(nullptr)
    , num_chunk_samples_(0)
    , is_started_(false) {
}

AudioProcessing::~AudioProcessing() {
    if (near_chan_buf_) delete static_cast<webrtc::ChannelBuffer<float>*>(near_chan_buf_);
    if (far_chan_buf_) delete static_cast<webrtc::ChannelBuffer<float>*>(far_chan_buf_);
    if (out_chan_buf_) delete static_cast<webrtc::ChannelBuffer<float>*>(out_chan_buf_);
    if (stream_config_in_) delete static_cast<webrtc::StreamConfig*>(stream_config_in_);
    if (stream_config_out_) delete static_cast<webrtc::StreamConfig*>(stream_config_out_);
    if (audio_processor_) delete static_cast<webrtc::AudioProcessing*>(audio_processor_);

}

bool AudioProcessing::setConfig(int configId, std::variant<int, bool, float> value) {
    switch (configId) {
        case SAMPLE_RATE:
            sample_rate_ = std::get<int>(value);
            break;
        case SYSTEM_DELAY_MS:
            system_delay_ms_ = std::get<int>(value);
            break;
        case NOISE_SUPPRESSION_LEVEL:
            noise_suppression_level_ = std::get<int>(value);
            break;
        case AEC_LEVEL:
            aec_level_ = std::get<int>(value);
            break;
        case ENABLE_AEC:
            enable_aec_ = std::get<bool>(value);
            break;
        case ENABLE_AGC:
            enable_agc_ = std::get<bool>(value);
            break;
        case ENABLE_HP_FILTER:
            enable_hp_filter_ = std::get<bool>(value);
            break;
        case ENABLE_NOISE_SUPPRESSION:
            enable_noise_suppression_ = std::get<bool>(value);
            break;
        case ENABLE_TRANSIENT_SUPPRESSION:
            enable_transient_suppression_ = std::get<bool>(value);
            break;
        case AEC_DELAY_AGNOSTIC:
            aec_delay_agnostic_ = std::get<bool>(value);
            break;
        case AEC_EXTENDED_FILTER:
            aec_extended_filter_ = std::get<bool>(value);
            break;
        case ENABLE_VOICE_DETECTION:
            enable_voice_detection_ = std::get<bool>(value);
            break;
        case AGC_MODE:
            agc_mode_ = std::get<int>(value);
            break;
        default:
            return false;
    }

    return true;
}

void AudioProcessing::start() {
    if (is_started_) return;

    audio_processor_ = webrtc::AudioProcessing::Create();
    if (!audio_processor_) {
        return;
    }

    configureProcessing();

    num_chunk_samples_ = sample_rate_ / 100;

    stream_config_in_ = new webrtc::StreamConfig(sample_rate_, WEBRTC_AEC3_NUM_CHANNELS);
    stream_config_out_ = new webrtc::StreamConfig(sample_rate_, WEBRTC_AEC3_NUM_CHANNELS);

    // Allocate float buffers
    near_float_data_.resize(num_chunk_samples_);
    far_float_data_.resize(num_chunk_samples_);
    out_float_data_.resize(num_chunk_samples_);

    // Create channel buffers
    near_chan_buf_ = new webrtc::ChannelBuffer<float>(num_chunk_samples_, WEBRTC_AEC3_NUM_CHANNELS, num_chunk_samples_);
    far_chan_buf_ = new webrtc::ChannelBuffer<float>(num_chunk_samples_, WEBRTC_AEC3_NUM_CHANNELS, num_chunk_samples_);
    out_chan_buf_ = new webrtc::ChannelBuffer<float>(num_chunk_samples_, WEBRTC_AEC3_NUM_CHANNELS, num_chunk_samples_);

    is_started_ = true;
}

void AudioProcessing::configureProcessing() {
    if (!audio_processor_) return;

    webrtc::AudioProcessing* processor = static_cast<webrtc::AudioProcessing*>(audio_processor_);

    // Configure Echo Cancellation (AEC)
    if (enable_aec_) {
        processor->echo_cancellation()->Enable(true);
        processor->echo_cancellation()->enable_drift_compensation(false);
        processor->echo_cancellation()->set_suppression_level(
            static_cast<webrtc::EchoCancellation::SuppressionLevel>(aec_level_));

        // Configure delay agnostic if supported
        if (aec_delay_agnostic_) {
            processor->echo_cancellation()->enable_delay_logging(true);
        }
    } else {
        processor->echo_cancellation()->Enable(false);
    }

    // Configure AGC
    if (enable_agc_) {
        processor->gain_control()->Enable(true);
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
        processor->gain_control()->set_mode(agc_webrtc_mode);
    } else {
        processor->gain_control()->Enable(false);
    }

    // Configure NS
    if (enable_noise_suppression_) {
        processor->noise_suppression()->Enable(true);
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
        processor->noise_suppression()->set_level(ns_level);
    } else {
        processor->noise_suppression()->Enable(false);
    }

    // Configure HPF
    if (enable_hp_filter_) {
        processor->high_pass_filter()->Enable(true);
    } else {
        processor->high_pass_filter()->Enable(false);
    }

    // Configure Voice Detection
    if (enable_voice_detection_) {
        processor->voice_detection()->Enable(true);
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
        processor->voice_detection()->set_likelihood(vad_likelihood);
    } else {
        processor->voice_detection()->Enable(false);
    }

    // Set stream delay if needed
    if (system_delay_ms_ > 0) {
        processor->set_stream_delay_ms(system_delay_ms_);
    }
}

void AudioProcessing::process(const std::vector<int16_t>& near_in,
                        const std::vector<int16_t>& far_in,
                        std::vector<int16_t>& out) {
    if (!is_started_ || !audio_processor_) {
        return;
    }

    validateInputSizes(near_in, far_in);

    // Ensure output vector is properly sized
    out.resize(near_in.size());

    // Cast void pointers to proper types
    webrtc::AudioProcessing* processor = static_cast<webrtc::AudioProcessing*>(audio_processor_);
    webrtc::StreamConfig* stream_in = static_cast<webrtc::StreamConfig*>(stream_config_in_);
    webrtc::StreamConfig* stream_out = static_cast<webrtc::StreamConfig*>(stream_config_out_);
    webrtc::ChannelBuffer<float>* near_buf = static_cast<webrtc::ChannelBuffer<float>*>(near_chan_buf_);
    webrtc::ChannelBuffer<float>* far_buf = static_cast<webrtc::ChannelBuffer<float>*>(far_chan_buf_);
    webrtc::ChannelBuffer<float>* out_buf = static_cast<webrtc::ChannelBuffer<float>*>(out_chan_buf_);

    // Process audio in chunks
    size_t samples_processed = 0;
    const size_t total_samples = near_in.size();

    while (samples_processed < total_samples) {
        const size_t samples_to_process = std::min(num_chunk_samples_, total_samples - samples_processed);


        for (size_t i = 0; i < samples_to_process; ++i) {
            near_float_data_[i] = static_cast<float>(near_in[samples_processed + i]) / 32768.0f;
            far_float_data_[i] = static_cast<float>(far_in[samples_processed + i]) / 32768.0f;
        }


        std::copy(near_float_data_.begin(), near_float_data_.begin() + samples_to_process,
                  near_buf->channels()[0]);
        std::copy(far_float_data_.begin(), far_float_data_.begin() + samples_to_process,
                  far_buf->channels()[0]);


        int result = processor->ProcessReverseStream(
            far_buf->channels(), *stream_in, *stream_out,
            far_buf->channels());

        if (result != webrtc::AudioProcessing::kNoError) {
            // Handle error
            continue;
        }

        // Process forward stream (near-end/microphone)
        result = processor->ProcessStream(
            near_buf->channels(), *stream_in, *stream_out,
            out_buf->channels());

        if (result != webrtc::AudioProcessing::kNoError) {
            // Handle error
            continue;
        }

        // Convert float back to int16_t
        for (size_t i = 0; i < samples_to_process; ++i) {
            float sample = out_buf->channels()[0][i];
            // Clamp to [-1.0, 1.0] and convert to int16_t
            sample = std::max(-1.0f, std::min(1.0f, sample));
            out[samples_processed + i] = static_cast<int16_t>(sample * 32767.0f);
        }

        samples_processed += samples_to_process;
    }
}

bool AudioProcessing::processRawBytes(const uint8_t* nearBytes, size_t nearByteCount,
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

    // Process the audio
    process(nearSamples, farSamples, out);
    return true;
}

void AudioProcessing::validateInputSizes(const std::vector<int16_t>& near_in,
                                   const std::vector<int16_t>& far_in) const {
    if (near_in.size() != far_in.size()) {
        throw std::invalid_argument("Near and far input sizes must match");
    }

    if (near_in.empty()) {
        throw std::invalid_argument("Input vectors cannot be empty");
    }
}

bool AudioProcessing::hasVoice() const {
    if (!audio_processor_ || !enable_voice_detection_) {
        return false;
    }

    webrtc::AudioProcessing* processor = static_cast<webrtc::AudioProcessing*>(audio_processor_);
    return processor->voice_detection()->stream_has_voice();
}

bool AudioProcessing::hasEcho() const {
    if (!audio_processor_ || !enable_aec_) {
        return false;
    }

    webrtc::AudioProcessing* processor = static_cast<webrtc::AudioProcessing*>(audio_processor_);
    // Check if echo cancellation is processing
    return processor->echo_cancellation()->is_enabled();
}

float AudioProcessing::getSpeechProbability() const {
    if (!audio_processor_ || !enable_voice_detection_) {
        return 0.0f;
    }
    return hasVoice() ? 1.0f : 0.0f;
}
