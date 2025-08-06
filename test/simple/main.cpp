#include "audio_processing.h"
#include <iostream>
#include <vector>

int main() {


        audio_processing aec_simple;

        // simple configuration
        aec_simple.setConfig(audio_processing::SAMPLE_RATE, 16000);
        aec_simple.setConfig(audio_processing::ENABLE_AEC, true);
        aec_simple.setConfig(audio_processing::ENABLE_NOISE_SUPPRESSION, true);
        aec_simple.setConfig(audio_processing::NOISE_SUPPRESSION_LEVEL, audio_processing::NS_LEVEL_MODERATE);
        aec_simple.start();

        // simple initialize
        std::vector<int16_t> near(160, 0);
        std::vector<int16_t> far(160, 0);
        std::vector<int16_t> out;

        aec_simple.process(near, far, out);

        std::cout << "Simple processing test completed. Output size: " << out.size() << std::endl;
        return 0;
}
