#include "ImagePointContainer.hpp"

#include <charconv>
#include <ranges>
#include <fstream>

#include <iostream>
#include <cmath>

using namespace multicamera_resection;

void ImagePointContainer::readDataFromFile(const std::filesystem::path& pathToFile)
{
    m_data.clear();
    m_data.reserve(16384);

    if (!std::filesystem::exists(pathToFile))
    {
        throw std::invalid_argument("Fatal Error. File with image points does not exist!");
    }

    auto file{std::ifstream{pathToFile}};

    for (std::string line; std::getline(file, line, '\n');)
    {
        std::istringstream dataRecord;
        dataRecord.str(line);
        std::vector<std::string> entries;
        entries.reserve(numberOfPointDataEntries);
        for (std::string entry; std::getline(dataRecord, entry, ',');)
        {
            entries.push_back(entry);
        }
        const auto pointData{tryParseData(entries)};
        m_data.emplace_back(pointData);
    }

    m_data.shrink_to_fit();
}

ImagePoint ImagePointContainer::tryParseData(const std::vector<std::string> &entries)
{
    if (entries.size() != numberOfPointDataEntries)
    {
        throw std::invalid_argument("Fatal error. Invalid number of entries for image points data! Data record is missformated!");
    }

    ImagePoint imagePoint;

    for (const auto [index, entry] : entries | std::views::enumerate | std::views::drop(2) | std::views::take(numberOfPointDataEntries-2))
    {
        auto value{0.0};

        if (std::from_chars(entry.c_str(), entry.c_str() + entry.length(), value).ec == std::errc()) [[likely]]
        {
            if (std::isnan(value) || std::isinf(value))
            {
                throw std::invalid_argument("Not a number (nan) or inifinite (inf) value detected in point data");
            }

            if (value < -0.5)
            {
                std::cout << "invalid value: " << value <<std::endl;
                throw std::invalid_argument("Invalid value detected for image points coordinates!");
            }
            
            if (index == 2)
            {
                imagePoint.x = value;
            }

            if (index == 3)
            {
                imagePoint.y = value;
            }

        }
        else
        {
            throw std::invalid_argument("Not a numeric entry found for image points data, or data record is missformated!");
        }

    }

    imagePoint.cameraId = entries[0];
    imagePoint.id = entries[1];
    

    return imagePoint;
}

void ImagePointContainer::sentToStream(std::ostream &outputStream) const
{
    for (const auto &point : m_data)
    {
        point.sentToStream(outputStream);
    }
}

size_t ImagePointContainer::size() const
{
    return m_data.size();
}