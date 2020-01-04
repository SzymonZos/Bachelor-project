#ifndef MPC_STM_DATAPARSER_HPP
#define MPC_STM_DATAPARSER_HPP

#include <map>
#include <vector>
#include <string>

namespace Utils {
    class DataParser {
    public:
        static bool isNewDataGoingToBeSend;

        DataParser() = default;
        ~DataParser() = default;
        void ParseReceivedMsg(const std::string& msg);

        [[nodiscard]] std::map<std::string, std::vector<double>> GetStorage() const;
        void ClearStorage();

    private:
        constexpr static const char* regexPattern = R"(([[:alpha:]]+)(': )(\[.+?\]))";
        std::map<std::string, std::vector<double>> storage;
    };
}


#endif //MPC_STM_DATAPARSER_HPP
