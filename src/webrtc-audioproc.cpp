#include "WebrtcAEC3.h"
#include "webrtc/modules/audio_processing/include/audio_processing.h"
#include "webrtc/modules/audio_processing/audio_buffer.h"
#include "webrtc/modules/audio_processing/echo_cancellation_impl.h"
#include "webrtc/modules/audio_processing/aec/aec_core_internal.h"
#include <iostream>
#include <stdexcept>
#include <cstring>

using namespace webrtc;

// Endianness detection and conversion helpers
namespace {
    // Check if system is little endian
    inline bool isLittleEndian() {
        static const uint16_t test = 0x0001;
        return *reinterpret_cast<const uint8_t*>(&test) == 0x01;
    }

    // Convert 16-bit value from little endian to host byte order
    inline int16_t le16ToHost(int16_t value) {
        if (isLittleEndian()) {
            return value;
        } else {
            // Big endian system - swap bytes
            return static_cast<int16_t>(((value & 0xFF) << 8) | ((value >> 8) & 0xFF));
        }
    }

    // Convert 16-bit value from host byte order to little endian
    inline int16_t hostToLe16(int16_t value) {
        return le16ToHost(value); // Same operation for conversion both ways
    }

    // Convert byte array to int16_t vector with endianness handling
    // Assumes input data is in little endian format (network standard)
    void bytesToInt16Vector(const uint8_t* bytes, size_t byteCount, std::vector<int16_t>& output) {
        if (byteCount % 2 != 0) {
            throw std::invalid_argument("Byte count must be even for int16_t conversion");
        }

        size_t sampleCount = byteCount / 2;
        output.resize(sampleCount);

        for (size_t i = 0; i < sampleCount; ++i) {
            // Read as little endian (network byte order)
            int16_t rawValue = static_cast<int16_t>(bytes[i * 2]) |
                              (static_cast<int16_t>(bytes[i * 2 + 1]) << 8);
            output[i] = le16ToHost(rawValue);
        }
    }

    // Convert int16_t vector to byte array with endianness handling
    // Outputs data in little endian format (network standard)
    void int16VectorToBytes(const std::vector<int16_t>& input, uint8_t* bytes) {
        for (size_t i = 0; i < input.size(); ++i) {
            int16_t value = hostToLe16(input[i]);
            bytes[i * 2] = static_cast<uint8_t>(value & 0xFF);
            bytes[i * 2 + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
        }
    }

    // Calculate expected samples based on sample rate and duration
    size_t calculateExpectedSamples(int sampleRate, int durationMs) {
        return static_cast<size_t>((sampleRate * durationMs) / 1000);
    }
}

WebrtcAEC3::WebrtcAEC3()
    : sample_rate_(48000)
    , system_delay_ms_(8)
    , noise_suppression_level_(2)
    , aec_level_(2)
    , enable_aec_(true)
    , enable_agc_(true)
    , enable_hp_filter_(true)
    , enable_noise_suppression_(true)
    , enable_transient_suppression_(true)
    , aec_delay_agnostic_(true)
    , aec_extended_filter_(false)
    , enable_voice_detection_(true)
    , audio_processor_(nullptr)
    , stream_config_in_(nullptr)
    , stream_config_out_(nullptr)
    , near_chan_buf_(nullptr)
    , far_chan_buf_(nullptr)
    , out_chan_buf_(nullptr)
    , num_chunk_samples_(0)
    , is_started_(false) {
}

WebrtcAEC3::~WebrtcAEC3() {
    delete audio_processor_;
    delete stream_config_in_;
    delete stream_config_out_;
    delete near_chan_buf_;
    delete far_chan_buf_;
    delete out_chan_buf_;
}

bool WebrtcAEC3::setConfig(int configId, ConfigVariant value) {
    switch (configId) {
    case SAMPLE_RATE:
        if (is_started_ || value.type() != ConfigVariant::INT) return false;
        sample_rate_ = value.as_int();
        return true;

    case SYSTEM_DELAY_MS:
        if (is_started_ || value.type() != ConfigVariant::INT) return false;
        system_delay_ms_ = value.as_int();
        return true;

    case NOISE_SUPPRESSION_LEVEL:
        if (value.type() != ConfigVariant::INT) return false;
        {
            int val = value.as_int();
            if (val < NS_LEVEL_LOW || val > NS_LEVEL_VERY_HIGH) return false;
            noise_suppression_level_ = val;
            if (is_started_ && audio_processor_) {
                audio_processor_->noise_suppression()->set_level(
                    static_cast<webrtc::NoiseSuppression::Level>(noise_suppression_level_));
            }
        }
        return true;

    case AEC_LEVEL:
        if (value.type() != ConfigVariant::INT) return false;
        aec_level_ = value.as_int();
        if (is_started_ && audio_processor_ && enable_aec_) {
            audio_processor_->echo_cancellation()->set_suppression_level(
                static_cast<webrtc::EchoCancellation::SuppressionLevel>(aec_level_));
        }
        return true;

    case ENABLE_AEC:
        if (value.type() != ConfigVariant::BOOL) return false;
        enable_aec_ = value.as_bool();
        if (is_started_ && audio_processor_) {
            audio_processor_->echo_cancellation()->Enable(enable_aec_);
        }
        return true;

    case ENABLE_AGC:
        if (value.type() != ConfigVariant::BOOL) return false;
        enable_agc_ = value.as_bool();
        if (is_started_ && audio_processor_) {
            audio_processor_->gain_control()->Enable(enable_agc_);
        }
        return true;

    case ENABLE_HP_FILTER:
        if (value.type() != ConfigVariant::BOOL) return false;
        enable_hp_filter_ = value.as_bool();
        if (is_started_ && audio_processor_) {
            audio_processor_->high_pass_filter()->Enable(enable_hp_filter_);
        }
        return true;

    case ENABLE_NOISE_SUPPRESSION:
        if (value.type() != ConfigVariant::BOOL) return false;
        enable_noise_suppression_ = value.as_bool();
        if (is_started_ && audio_processor_) {
            audio_processor_->noise_suppression()->Enable(enable_noise_suppression_);
        }
        return true;

    case ENABLE_TRANSIENT_SUPPRESSION:
        if (value.type() != ConfigVariant::BOOL) return false;
        enable_transient_suppression_ = value.as_bool();
        return true;

    case AEC_DELAY_AGNOSTIC:
        if (is_started_ || value.type() != ConfigVariant::BOOL) return false;
        aec_delay_agnostic_ = value.as_bool();
        return true;

    case AEC_EXTENDED_FILTER:
        if (is_started_ || value.type() != ConfigVariant::BOOL) return false;
        aec_extended_filter_ = value.as_bool();
        return true;

    case ENABLE_VOICE_DETECTION:
        if (value.type() != ConfigVariant::BOOL) return false;
        enable_voice_detection_ = value.as_bool();
        if (is_started_ && audio_processor_) {
            audio_processor_->voice_detection()->Enable(enable_voice_detection_);
        }
        return true;

    case AGC_MODE:
        if (value.type() != ConfigVariant::INT) return false;
        {
            int val = value.as_int();
            if (val < AGC_MODE_ADAPTIVE_ANALOG || val > AGC_MODE_FIXED_DIGITAL)
                return false;
            agc_mode_ = val;
            if (is_started_ && audio_processor_) {
                audio_processor_->gain_control()->set_mode(
                    static_cast<webrtc::GainControl::Mode>(agc_mode_));
            }
        }
        return true;

    default:
        return false;
    }
}

void WebrtcAEC3::start() {
    if (is_started_) {
        // Already started
        return;
    }

    // Calculate chunk size (10ms worth of samples)
    num_chunk_samples_ = sample_rate_ / 100;

    // Initialize buffers
    near_float_data_.resize(num_chunk_samples_ * WEBRTC_AEC3_NUM_CHANNELS);
    far_float_data_.resize(num_chunk_samples_ * WEBRTC_AEC3_NUM_CHANNELS);
    out_float_data_.resize(num_chunk_samples_ * WEBRTC_AEC3_NUM_CHANNELS);

    // Initialize channel buffers
    near_chan_buf_ = new ChannelBuffer<float>(num_chunk_samples_, WEBRTC_AEC3_NUM_CHANNELS);
    far_chan_buf_ = new ChannelBuffer<float>(num_chunk_samples_, WEBRTC_AEC3_NUM_CHANNELS);
    out_chan_buf_ = new ChannelBuffer<float>(num_chunk_samples_, WEBRTC_AEC3_NUM_CHANNELS);

    // Initialize stream configs
    stream_config_in_ = new StreamConfig(sample_rate_, WEBRTC_AEC3_NUM_CHANNELS);
    stream_config_out_ = new StreamConfig(sample_rate_, WEBRTC_AEC3_NUM_CHANNELS);

    // Configure audio processing
    configureProcessing();

    is_started_ = true;
}

void WebrtcAEC3::configureProcessing() {
    // Create base configuration
    Config config;
    config.Set<ExperimentalNs>(new ExperimentalNs(enable_transient_suppression_));

    // Create AudioProcessing instance
    audio_processor_ = AudioProcessing::Create(config);

    // Set extra configuration options
    Config extraconfig;
    extraconfig.Set<DelayAgnostic>(new DelayAgnostic(aec_delay_agnostic_));
    extraconfig.Set<ExtendedFilter>(new ExtendedFilter(aec_extended_filter_));
    extraconfig.Set<ExtendedFilter>(new ExtendedFilter(enable_voice_detection_));
    extraconfig.Set<EchoCanceller3>(new EchoCanceller3(true));
    audio_processor_->SetExtraOptions(extraconfig);

    // Configure Echo Cancellation
    RTC_CHECK_EQ(AudioProcessing::kNoError, audio_processor_->echo_cancellation()->Enable(enable_aec_));
    if (enable_aec_) {
        audio_processor_->echo_cancellation()->set_suppression_level(EchoCancellation::kHighSuppression);
        if (aec_level_ != -1) {
            RTC_CHECK_EQ(AudioProcessing::kNoError,
                         audio_processor_->echo_cancellation()->set_suppression_level(
                             static_cast<EchoCancellation::SuppressionLevel>(aec_level_)));
        }
        audio_processor_->echo_cancellation()->enable_metrics(true);
        audio_processor_->echo_cancellation()->enable_delay_logging(true);
    }

    // Configure Noise Suppression
    RTC_CHECK_EQ(AudioProcessing::kNoError, audio_processor_->noise_suppression()->Enable(enable_noise_suppression_));

    if (enable_noise_suppression_) {
        if (noise_suppression_level_ < NS_LEVEL_LOW || noise_suppression_level_ > NS_LEVEL_VERY_HIGH) {
            noise_suppression_level_ = NS_LEVEL_MODERATE;
            std::cerr << "[NS] Invalid level provided. Falling back to MODERATE." << std::endl;
        }

        RTC_CHECK_EQ(AudioProcessing::kNoError, audio_processor_->noise_suppression()->set_level(
                         static_cast<NoiseSuppression::Level>(noise_suppression_level_)));

        std::cout << "[NS] Noise Suppression enabled. Level: " << noise_suppression_level_ << std::endl;
    } else {
        std::cout << "[NS] Noise Suppression disabled." << std::endl;
    }

    // Configure High Pass Filter
    RTC_CHECK_EQ(AudioProcessing::kNoError, audio_processor_->high_pass_filter()->Enable(enable_hp_filter_));

    // Configure Automatic Gain Control
    RTC_CHECK_EQ(AudioProcessing::kNoError, audio_processor_->gain_control()->Enable(enable_agc_));

    if (enable_agc_) {
        if (agc_mode_ < AGC_MODE_ADAPTIVE_ANALOG || agc_mode_ > AGC_MODE_FIXED_DIGITAL) {
            agc_mode_ = AGC_MODE_ADAPTIVE_DIGITAL;
            std::cerr << "[AGC] Invalid mode provided. Falling back to ADAPTIVE_DIGITAL." << std::endl;
        }

        RTC_CHECK_EQ(AudioProcessing::kNoError,audio_processor_->gain_control()->set_mode(
                         static_cast<GainControl::Mode>(agc_mode_)));
        //------------------------
        // AGC setting (optional)
        audio_processor_->gain_control()->set_target_level_dbfs(3);
        audio_processor_->gain_control()->set_compression_gain_db(9);
        audio_processor_->gain_control()->enable_limiter(true);
        audio_processor_->gain_control()->Enable(true);
        audio_processor_->gain_control()->set_mode(static_cast<GainControl::Mode>(agc_mode_));
        //----------------------
        std::cout << "[AGC] Enabled. Mode: " << agc_mode_ << std::endl;
    } else {
        std::cout << "[AGC] Disabled." << std::endl;
    }

    // Configure Voice Detection
    if (enable_voice_detection_) {
        audio_processor_->voice_detection()->Enable(true);
        audio_processor_->voice_detection()->set_likelihood(VoiceDetection::kVeryLowLikelihood);
        audio_processor_->voice_detection()->set_frame_size_ms(10);
    }
}

void WebrtcAEC3::validateInputSizes(const std::vector<int16_t>& near_in,
                                    const std::vector<int16_t>& far_in) const {
    if (near_in.size() != num_chunk_samples_) {
        throw std::invalid_argument("near_in size (" + std::to_string(near_in.size()) +
                                    ") does not match expected size (" + std::to_string(num_chunk_samples_) + ")");
    }
    if (far_in.size() != num_chunk_samples_) {
        throw std::invalid_argument("far_in size (" + std::to_string(far_in.size()) +
                                    ") does not match expected size (" + std::to_string(num_chunk_samples_) + ")");
    }
}

// New method to process raw byte data with endianness handling
bool WebrtcAEC3::processRawBytes(const uint8_t* nearBytes, size_t nearByteCount,
                                 const uint8_t* farBytes, size_t farByteCount,
                                 std::vector<int16_t>& out) {
    if (!is_started_) {
        throw std::runtime_error("WebrtcAEC3 must be started before processing");
    }

    // Calculate expected sample count for current chunk (10ms)
    size_t expectedSamples = num_chunk_samples_;
    size_t expectedByteCount = expectedSamples * sizeof(int16_t);

    // Validate input sizes - they should match the expected chunk size
    if (nearByteCount != expectedByteCount) {
        std::cerr << "Warning: near audio byte count (" << nearByteCount
                  << ") doesn't match expected (" << expectedByteCount
                  << ") for " << expectedSamples << " samples at " << sample_rate_ << "Hz" << std::endl;
        return false;
    }

    if (farByteCount != expectedByteCount) {
        std::cerr << "Warning: far audio byte count (" << farByteCount
                  << ") doesn't match expected (" << expectedByteCount
                  << ") for " << expectedSamples << " samples at " << sample_rate_ << "Hz" << std::endl;
        return false;
    }

    try {
        // Convert byte arrays to int16 vectors with proper endianness handling
        std::vector<int16_t> nearAudio, farAudio;
        bytesToInt16Vector(nearBytes, nearByteCount, nearAudio);
        bytesToInt16Vector(farBytes, farByteCount, farAudio);

        // Process the audio
        process(nearAudio, farAudio, out);
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "Error processing raw bytes: " << e.what() << std::endl;
        return false;
    }
}


void WebrtcAEC3::process(const std::vector<int16_t>& near_in,
                         const std::vector<int16_t>& far_in,
                         std::vector<int16_t>& out) {
    if (!is_started_) {
        throw std::runtime_error("WebrtcAEC3 must be started before processing");
    }

    validateInputSizes(near_in, far_in);

    // Resize output vector
    out.resize(num_chunk_samples_);

    // Convert far-end input from int16 to float
    S16ToFloat(far_in.data(), far_in.size(), far_float_data_.data());
    // Since we're mono, no deinterleaving needed - just copy to channel buffer
    std::copy(far_float_data_.begin(), far_float_data_.end(), far_chan_buf_->channels()[0]);

    // Convert near-end input from int16 to float
    S16ToFloat(near_in.data(), near_in.size(), near_float_data_.data());
    // Since we're mono, no deinterleaving needed - just copy to channel buffer
    std::copy(near_float_data_.begin(), near_float_data_.end(), near_chan_buf_->channels()[0]);

    // Set system delay
    RTC_CHECK_EQ(AudioProcessing::kNoError,
                 audio_processor_->set_stream_delay_ms(system_delay_ms_));

    // Process reverse stream (far-end/reference signal)
    RTC_CHECK_EQ(AudioProcessing::kNoError,
                 audio_processor_->ProcessReverseStream(far_chan_buf_->channels(),
                                                        *stream_config_in_,
                                                        *stream_config_out_,
                                                        far_chan_buf_->channels()));

    // Process microphone signal (near-end audio) with echo cancellation
    RTC_CHECK_EQ(AudioProcessing::kNoError,
                 audio_processor_->ProcessStream(near_chan_buf_->channels(),
                                                 *stream_config_in_,
                                                 *stream_config_out_,
                                                 out_chan_buf_->channels()));

    // Since we're mono, no interleaving needed - just copy from channel buffer
    std::copy(out_chan_buf_->channels()[0],
            out_chan_buf_->channels()[0] + num_chunk_samples_,
            out_float_data_.begin());

    // Convert output from float to int16
    FloatToS16(out_float_data_.data(), out.size(), out.data());
}

// Voice activity detection result
bool WebrtcAEC3::hasVoice() const {
    if (!is_started_ || !enable_voice_detection_) {
        return false;
    }
    return audio_processor_->voice_detection()->stream_has_voice();
}

// Echo detection result
bool WebrtcAEC3::hasEcho() const {
    if (!is_started_ || !enable_aec_) {
        return false;
    }
    return audio_processor_->echo_cancellation()->stream_has_echo();
}

// Speech probability from noise suppression
float WebrtcAEC3::getSpeechProbability() const {
    if (!is_started_ || !enable_noise_suppression_) {
        return 0.0f;
    }
    return audio_processor_->noise_suppression()->speech_probability();
}
