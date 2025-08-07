#include "audio_processing.h"
#include <iostream>
#include <vector>
#include <variant>

int main() {


        AudioProcessing aec_simple;

        // simple configuration
        aec_simple.setConfig(AudioProcessing::SAMPLE_RATE, 16000);
        aec_simple.setConfig(AudioProcessing::ENABLE_AEC, true);
        aec_simple.setConfig(AudioProcessing::ENABLE_NOISE_SUPPRESSION, true);
        aec_simple.setConfig(AudioProcessing::NOISE_SUPPRESSION_LEVEL, AudioProcessing::NS_LEVEL_MODERATE);
        aec_simple.start();

        // simple initialize
        std::vector<int16_t> near(160, 0);
        std::vector<int16_t> far(160, 0);
        std::vector<int16_t> out;

        aec_simple.process(near, far, out);

        std::cout << "Simple processing test completed. Output size: " << out.size() << std::endl;
        return 0;
}
