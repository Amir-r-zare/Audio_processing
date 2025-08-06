#include "audio_processing.h"
#include <iostream>
#include <vector>

int main() {
    try {
        WebrtcAEC3 aec;

        // simple configuration
        aec.setConfig(WebrtcAEC3::SAMPLE_RATE, 16000);
        aec.setConfig(WebrtcAEC3::ENABLE_AEC, true);
        aec.setConfig(WebrtcAEC3::ENABLE_NOISE_SUPPRESSION, true);
        aec.setConfig(WebrtcAEC3::NOISE_SUPPRESSION_LEVEL, WebrtcAEC3::NS_LEVEL_MODERATE);
//        aec.setConfig(WebrtcAEC3::AGC_MODE, ConfigVariant(5));
        aec.start();

        // simple initialize
        std::vector<int16_t> near(160, 0);
        std::vector<int16_t> far(160, 0);
        std::vector<int16_t> out;

        aec.process(near, far, out);

        std::cout << "Simple processing test completed. Output size: " << out.size() << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error during test: " << e.what() << std::endl;
        return 1;
    }
}


//int main()
//{
//    return 1;
//}
