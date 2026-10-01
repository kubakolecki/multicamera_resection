#pragma once

#include "ImagePoint.hpp"

#include <vector>
#include <filesystem>

namespace multicamera_resection
{

static constexpr size_t numberOfPointDataEntries{4uz};    

class ImagePointContainer
{

    public:
        const std::vector<ImagePoint>& getData() const
        {
            return m_data;
        }

        void readDataFromFile(const std::filesystem::path& pathToFile);
        void sentToStream(std::ostream &outputStream) const;
        size_t size() const;

    private:
        std::vector<ImagePoint> m_data;
        ImagePoint tryParseData(const std::vector<std::string> &entries);

};

}