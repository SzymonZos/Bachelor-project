#include "utils/DataParser.hpp"
#include "utils/Misc.hpp"
#include <regex>
#include <algorithm>


bool Utils::DataParser::isNewDataGoingToBeSend = false;


std::map<std::string, std::vector<double>> Utils::DataParser::GetStorage() const {
    return storage;
}


void Utils::DataParser::ParseReceivedMsg(const std::string& msg) {
    std::string valuesMatch;
    std::regex pattern(regexPattern);
    std::smatch fullMatch;
    std::string::const_iterator iterator(msg.cbegin());

    while (std::regex_search(iterator, msg.cend(), fullMatch, pattern)) {
        valuesMatch = fullMatch[3].str();
        std::replace(valuesMatch.begin(), valuesMatch.end(), ',', ' ');
        valuesMatch.pop_back(); // trim ]
        valuesMatch.erase(0, 1); // trim [
        if (fullMatch[1].str().find('A') != std::string::npos) {
            Utils::Misc::StringToDouble(valuesMatch, storage["A"]);
        } else if (fullMatch[1].str().find('B') != std::string::npos) {
            Utils::Misc::StringToDouble(valuesMatch, storage["B"]);
        } else if (fullMatch[1].str().find('C') != std::string::npos) {
            Utils::Misc::StringToDouble(valuesMatch, storage["C"]);
        } else if (fullMatch[1].str().find("set") != std::string::npos) {
            Utils::Misc::StringToDouble(valuesMatch, storage["set"]);
        } else if (fullMatch[1].str().find("control") != std::string::npos) {
            Utils::Misc::StringToDouble(valuesMatch, storage["control"]);
        } else if (fullMatch[1].str().find("horizon") != std::string::npos) {
            Utils::Misc::StringToDouble(valuesMatch, storage["horizons"]);
        }
        iterator = fullMatch.suffix().first;
    }
}


void Utils::DataParser::ClearStorage() {
    storage.clear();
}


