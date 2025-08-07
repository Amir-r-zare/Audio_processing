#include "audio_processing.h"
#include <iostream>
#include <vector>

int main() {
    AudioProcessing processor;
    processor.setConfig(AudioProcessing::SAMPLE_RATE, 16000);
    processor.setConfig(AudioProcessing::ENABLE_AEC, true);
    processor.setConfig(AudioProcessing::ENABLE_NOISE_SUPPRESSION, true);
    processor.setConfig(AudioProcessing::NOISE_SUPPRESSION_LEVEL, AudioProcessing::NS_LEVEL_HIGH);
    processor.setConfig(AudioProcessing::ENABLE_VOICE_DETECTION, true);

    processor.start();

    std::vector<int16_t> near(160, 1000);
    std::vector<int16_t> far(160, 500);
    std::vector<int16_t> out;
    processor.Process(near, far, out);
    std::cout << "Has Voice: " << std::boolalpha << processor.hasVoice()<< "\n";
    std::cout << "Has Echo: " << std::boolalpha << processor.hasEcho() << "\n";
    std::cout << "Speech Probability: " << processor.getSpeechProbability() <<"\n";

    if (!out.empty()) {
        std::cout << "First Output Sample: " << out[0] << "\n";
    } else {
        std::cout << "Output is empty." << "\n";
    }

    return 0;
}
