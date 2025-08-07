#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H

#include <vector>
#include <cstdint>
#include <memory>
#include <variant>

#define WEBRTC_AEC3_NUM_CHANNELS 1

class AudioProcessing {
public:
    AudioProcessing();
    ~AudioProcessing();

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

    // Combined method to process audio
    bool Process(const uint8_t* nearBytes, size_t nearByteCount,
                 const uint8_t* farBytes, size_t farByteCount,
                 std::vector<int16_t>& out);

    bool hasVoice() const;
    bool hasEcho() const;
    float getSpeechProbability() const;

private:
    class AudioProcessingPrivate;
    std::unique_ptr<AudioProcessingPrivate> pImpl;
};

#endif // AUDIO_PROCESSING_H
