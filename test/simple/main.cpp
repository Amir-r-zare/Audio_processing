#include <cmath>
#include <vector>
#include <cstdint>
#include <iostream>
#include "audio_processing.h"

constexpr int sampleRate = 16000;
constexpr float frequency = 440.0f; // A4 note
constexpr int durationMs = 100; // 100ms test
constexpr int totalSamples = (sampleRate * durationMs) / 1000;

void generateTestSignal(std::vector<uint8_t>& outBytes) {
    std::vector<int16_t> samples(totalSamples);
    for (int i = 0; i < totalSamples; ++i) {
        float t = static_cast<float>(i) / sampleRate;
        float sample = std::sin(2.0f * M_PI * frequency * t);
        samples[i] = static_cast<int16_t>(sample * 32767);
    }

    outBytes.resize(samples.size() * 2);
    for (size_t i = 0; i < samples.size(); ++i) {
        outBytes[2 * i]     = samples[i] & 0xFF;
        outBytes[2 * i + 1] = (samples[i] >> 8) & 0xFF;
    }
}

int main() {
    std::vector<uint8_t> nearBytes;
    std::vector<uint8_t> farBytes;
    generateTestSignal(nearBytes);
    generateTestSignal(farBytes);

    AudioProcessing processor;
    processor.setConfig(AudioProcessing::SAMPLE_RATE, 16000);
    processor.setConfig(AudioProcessing::ENABLE_AEC, true);
    processor.setConfig(AudioProcessing::ENABLE_NOISE_SUPPRESSION, true);
    processor.setConfig(AudioProcessing::NOISE_SUPPRESSION_LEVEL, AudioProcessing::NS_LEVEL_HIGH);
    processor.setConfig(AudioProcessing::ENABLE_VOICE_DETECTION, true);
    processor.start();

    std::vector<int16_t> output;
    bool result = processor.Process(nearBytes.data(), nearBytes.size(), farBytes.data(), farBytes.size(), output);

    if (result) {
        std::cout << "Process successful. Output size: " << output.size() << "\n";
    } else {
        std::cerr << "Process failed.\n";
    }

    return 0;
}




//#include "audio_processing.h"
//#include <iostream>
//#include <vector>

//int main() {
//    AudioProcessing processor;
//    processor.setConfig(AudioProcessing::SAMPLE_RATE, 16000);
//    processor.setConfig(AudioProcessing::ENABLE_AEC, true);
//    processor.setConfig(AudioProcessing::ENABLE_NOISE_SUPPRESSION, true);
//    processor.setConfig(AudioProcessing::NOISE_SUPPRESSION_LEVEL, AudioProcessing::NS_LEVEL_HIGH);
//    processor.setConfig(AudioProcessing::ENABLE_VOICE_DETECTION, true);
//    processor.start();

//    std::vector<int16_t> near(160, 1000);
//    std::vector<int16_t> far(160, 500);
//    std::vector<int16_t> out;

//    processor.Process()

//    std::cout << "Has Voice: " << std::boolalpha << processor.hasVoice()<< "\n";
//    std::cout << "Has Echo: " << std::boolalpha << processor.hasEcho() << "\n";
//    std::cout << "Speech Probability: " << processor.getSpeechProbability() <<"\n";
//    if (!out.empty()) {
//        std::cout << "First Output Sample: " << out[0] << "\n";
//    } else {
//        std::cout << "Output is empty." << "\n";
//    }

//    return 0;
//}
