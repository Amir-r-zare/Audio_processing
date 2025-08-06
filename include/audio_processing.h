#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H

#include <vector>
#include <cstdint>
#include <stdexcept>
#include <algorithm>
#include <memory>
#include <variant>

// Constants
#define WEBRTC_AEC3_NUM_CHANNELS 1

class audio_processing {
public:
    audio_processing();
    ~audio_processing();

    // Configuration IDs
    enum ConfigId {
        SAMPLE_RATE = 0,
        SYSTEM_DELAY_MS = 1,
        NOISE_SUPPRESSION_LEVEL = 2,
        AEC_LEVEL = 3,
        ENABLE_AEC = 4,
        ENABLE_AGC = 5,
        ENABLE_HP_FILTER = 6,
        ENABLE_NOISE_SUPPRESSION = 7,
        ENABLE_TRANSIENT_SUPPRESSION = 8,
        AEC_DELAY_AGNOSTIC = 9,
        AEC_EXTENDED_FILTER = 10,
        ENABLE_VOICE_DETECTION = 11,
        AGC_MODE = 12,
    };

    enum AgcMode {
        AGC_MODE_ADAPTIVE_ANALOG = 0,
        AGC_MODE_ADAPTIVE_DIGITAL = 1,
        AGC_MODE_FIXED_DIGITAL = 2
    };

    enum NoiseSuppressionLevel {
        NS_LEVEL_LOW = 0,
        NS_LEVEL_MODERATE = 1,
        NS_LEVEL_HIGH = 2,
        NS_LEVEL_VERY_HIGH = 3
    };

    enum VoiceDetectionLevel {
        kLowLikelihood = 0,
        kModerateLikelihood = 1,
        kHighLikelihood = 2
    };

    bool setConfig(int configId, std::variant<int, bool, float> value);
    void start();

    // Original process method for int16_t vectors
    void process(const std::vector<int16_t>& near_in,
                 const std::vector<int16_t>& far_in,
                 std::vector<int16_t>& out);

    // New methods for handling raw byte data with endianness conversion
    bool processRawBytes(const uint8_t* nearBytes, size_t nearByteCount,
                        const uint8_t* farBytes, size_t farByteCount,
                        std::vector<int16_t>& out);

    // Template method for processing byte containers (like QByteArray)
    template<typename ByteContainer>
    bool processRawData(const ByteContainer& nearData, const ByteContainer& farData,
                       std::vector<int16_t>& out);

    bool hasVoice() const;
    bool hasEcho() const;
    float getSpeechProbability() const;

    // Public config fields (if needed externally)
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

private:
    void configureProcessing();
    void validateInputSizes(const std::vector<int16_t>& near_in,
                            const std::vector<int16_t>& far_in) const;

    // WebRTC objects - using raw pointers instead of smart pointers
    void* audio_processor_;
    void* stream_config_in_;
    void* stream_config_out_;

    std::vector<float> near_float_data_;
    std::vector<float> far_float_data_;
    std::vector<float> out_float_data_;
    void* near_chan_buf_;
    void* far_chan_buf_;
    void* out_chan_buf_;

    size_t num_chunk_samples_;
    bool is_started_;
};

// Template method implementation (must be in header for templates)
template<typename ByteContainer>
bool audio_processing::processRawData(const ByteContainer& nearData, const ByteContainer& farData,
                               std::vector<int16_t>& out) {
    return processRawBytes(
        reinterpret_cast<const uint8_t*>(nearData.constData()), nearData.size(),
        reinterpret_cast<const uint8_t*>(farData.constData()), farData.size(),
        out
    );
}

#endif // AUDIO_PROCESSING_H


//#ifndef AUDIO_PROCESSING_H
//#define AUDIO_PROCESSING_H

//#include <vector>
//#include <cstdint>
//#include <stdexcept>
//#include <algorithm>
//#include <memory>
//#include <variant>

//// Constants
//#define WEBRTC_AEC3_NUM_CHANNELS 1

//// Forward declarations for WebRTC types
//namespace webrtc {
//    class AudioProcessing;
//    template<typename T> class ChannelBuffer;
//    struct StreamConfig;
//}

//class audio_processing {
//public:
//    audio_processing();
//    ~audio_processing();

//    // Configuration IDs
//    enum ConfigId {
//        SAMPLE_RATE = 0,
//        SYSTEM_DELAY_MS = 1,
//        NOISE_SUPPRESSION_LEVEL = 2,
//        AEC_LEVEL = 3,
//        ENABLE_AEC = 4,
//        ENABLE_AGC = 5,
//        ENABLE_HP_FILTER = 6,
//        ENABLE_NOISE_SUPPRESSION = 7,
//        ENABLE_TRANSIENT_SUPPRESSION = 8,
//        AEC_DELAY_AGNOSTIC = 9,
//        AEC_EXTENDED_FILTER = 10,
//        ENABLE_VOICE_DETECTION = 11,
//        AGC_MODE = 12,
//    };

//    enum AgcMode {
//        AGC_MODE_ADAPTIVE_ANALOG = 0,
//        AGC_MODE_ADAPTIVE_DIGITAL = 1,
//        AGC_MODE_FIXED_DIGITAL = 2
//    };

//    enum NoiseSuppressionLevel {
//        NS_LEVEL_LOW = 0,
//        NS_LEVEL_MODERATE = 1,
//        NS_LEVEL_HIGH = 2,
//        NS_LEVEL_VERY_HIGH = 3
//    };

//    enum VoiceDetectionLevel {
//        kLowLikelihood = 0,
//        kModerateLikelihood = 1,
//        kHighLikelihood = 2
//    };

//    bool setConfig(int configId, std::variant<int, bool, float> value);
//    void start();

//    // Original process method for int16_t vectors
//    void process(const std::vector<int16_t>& near_in,
//                 const std::vector<int16_t>& far_in,
//                 std::vector<int16_t>& out);

//    // New methods for handling raw byte data with endianness conversion
//    bool processRawBytes(const uint8_t* nearBytes, size_t nearByteCount,
//                        const uint8_t* farBytes, size_t farByteCount,
//                        std::vector<int16_t>& out);

//    // Template method for processing byte containers (like QByteArray)
//    template<typename ByteContainer>
//    bool processRawData(const ByteContainer& nearData, const ByteContainer& farData,
//                       std::vector<int16_t>& out);

//    bool hasVoice() const;
//    bool hasEcho() const;
//    float getSpeechProbability() const;

//    // Public config fields (if needed externally)
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

//private:
//    void configureProcessing();
//    void validateInputSizes(const std::vector<int16_t>& near_in,
//                            const std::vector<int16_t>& far_in) const;

//    // WebRTC objects - using raw pointers instead of smart pointers
//    webrtc::AudioProcessing* audio_processor_;
//    webrtc::StreamConfig* stream_config_in_;
//    webrtc::StreamConfig* stream_config_out_;

//    std::vector<float> near_float_data_;
//    std::vector<float> far_float_data_;
//    std::vector<float> out_float_data_;
//    webrtc::ChannelBuffer<float>* near_chan_buf_;
//    webrtc::ChannelBuffer<float>* far_chan_buf_;
//    webrtc::ChannelBuffer<float>* out_chan_buf_;

//    size_t num_chunk_samples_;
//    bool is_started_;
//};

//// Template method implementation (must be in header for templates)
//template<typename ByteContainer>
//bool audio_processing::processRawData(const ByteContainer& nearData, const ByteContainer& farData,
//                               std::vector<int16_t>& out) {
//    return processRawBytes(
//        reinterpret_cast<const uint8_t*>(nearData.constData()), nearData.size(),
//        reinterpret_cast<const uint8_t*>(farData.constData()), farData.size(),
//        out
//    );
//}

//#endif // AUDIO_PROCESSING_H



//#ifndef AUDIO_PROCESSING_H
//#define AUDIO_PROCESSING_H

//#include <vector>
//#include <cstdint>
//#include <stdexcept>
//#include <algorithm>
//#include <memory>
//#include <variant>

//// Constants
//#define WEBRTC_AEC3_NUM_CHANNELS 1

//// Simple variant implementation for C++11/14 compatibility
//class ConfigVariant {
//public:
//    enum Type { INT, BOOL, FLOAT };

//private:
//    Type type_;
//    union {
//        int int_val_;
//        bool bool_val_;
//        float float_val_;
//    };

//public:
//    ConfigVariant(int val) : type_(INT), int_val_(val) {}
//    ConfigVariant(bool val) : type_(BOOL), bool_val_(val) {}
//    ConfigVariant(float val) : type_(FLOAT), float_val_(val) {}

//    Type type() const { return type_; }
//    int as_int() const { return int_val_; }
//    bool as_bool() const { return bool_val_; }
//    float as_float() const { return float_val_; }
//};

//// Forward declarations for WebRTC types
//namespace webrtc {
//    class AudioProcessing;
//    template<typename T> class ChannelBuffer;
//    struct StreamConfig;
//}

//class audio_processing {
//public:

//    // Configuration IDs
//    enum ConfigId {
//        SAMPLE_RATE = 0,
//        SYSTEM_DELAY_MS = 1,
//        NOISE_SUPPRESSION_LEVEL = 2,
//        AEC_LEVEL = 3,
//        ENABLE_AEC = 4,
//        ENABLE_AGC = 5,
//        ENABLE_HP_FILTER = 6,
//        ENABLE_NOISE_SUPPRESSION = 7,
//        ENABLE_TRANSIENT_SUPPRESSION = 8,
//        AEC_DELAY_AGNOSTIC = 9,
//        AEC_EXTENDED_FILTER = 10,
//        ENABLE_VOICE_DETECTION = 11,
//        AGC_MODE = 12,
//    };

//    enum AgcMode {
//        AGC_MODE_ADAPTIVE_ANALOG = 0,
//        AGC_MODE_ADAPTIVE_DIGITAL = 1,
//        AGC_MODE_FIXED_DIGITAL = 2
//    };

//    enum NoiseSuppressionLevel {
//        NS_LEVEL_LOW = 0,
//        NS_LEVEL_MODERATE = 1,
//        NS_LEVEL_HIGH = 2,
//        NS_LEVEL_VERY_HIGH = 3
//    };

//    enum VoiceDetectionLevel {
//        kLowLikelihood = 0,
//        kModerateLikelihood = 1,
//        kHighLikelihood = 2
//    };

//    audio_processing();
//    ~audio_processing();

//    bool setConfig(int configId, std::variant<int, bool, float> value);
//    void start();

//    void process(const std::vector<int16_t>& near_in,
//                 const std::vector<int16_t>& far_in,
//                 std::vector<int16_t>& out);

//    bool processRawBytes(const uint8_t* nearBytes, size_t nearByteCount,
//                        const uint8_t* farBytes, size_t farByteCount,
//                        std::vector<int16_t>& out);

//    template<typename ByteContainer>
//    bool processRawData(const ByteContainer& nearData, const ByteContainer& farData,
//                       std::vector<int16_t>& out);

//    bool hasVoice() const;
//    bool hasEcho() const;
//    float getSpeechProbability() const;

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

//private:
//    void configureProcessing();
//    void validateInputSizes(const std::vector<int16_t>& near_in,
//                            const std::vector<int16_t>& far_in) const;

//    // WebRTC objects - using raw pointers instead of smart pointers
//    webrtc::AudioProcessing* audio_processor_;
//    webrtc::StreamConfig* stream_config_in_;
//    webrtc::StreamConfig* stream_config_out_;

//    std::vector<float> near_float_data_;
//    std::vector<float> far_float_data_;
//    std::vector<float> out_float_data_;
//    webrtc::ChannelBuffer<float>* near_chan_buf_;
//    webrtc::ChannelBuffer<float>* far_chan_buf_;
//    webrtc::ChannelBuffer<float>* out_chan_buf_;

//    size_t num_chunk_samples_;
//    bool is_started_;
//};


//template<typename ByteContainer>
//bool audio_processing::processRawData(const ByteContainer& nearData, const ByteContainer& farData,
//                               std::vector<int16_t>& out) {
//    return processRawBytes(
//        reinterpret_cast<const uint8_t*>(nearData.constData()), nearData.size(),
//        reinterpret_cast<const uint8_t*>(farData.constData()), farData.size(),
//        out
//    );
//}

//#endif // AUDIO_PROCESSING_H
