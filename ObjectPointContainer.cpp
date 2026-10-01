#include "ObjectPointContainer.hpp"

#include <charconv>
#include <ranges>
#include <fstream>
#include <iostream>

using namespace multicamera_resection;

void ObjectPointContainer::readDataFromFile(const std::filesystem::path& pathToFile)
{
    m_data.clear();

    if (!std::filesystem::exists(pathToFile))
    {
        throw std::invalid_argument("Fatal Error. File with camera data does not exist!");
    }

    auto file{std::ifstream{pathToFile}};
    std::string header;
    std::getline(file, header, '\n');

    for (std::string line; std::getline(file, line, '\n');)
    {
        std::istringstream dataRecord;
        dataRecord.str(line);
        std::vector<std::string> entries;
        entries.reserve(numberOfObjectPointEntries);
        for (std::string entry; std::getline(dataRecord, entry, ',');)
        {
            entries.push_back(entry);
        }
        const auto pointDataWithId{tryParsePointData(entries)};
        m_data.insert(pointDataWithId);
    }
}


ObjectPointWitId ObjectPointContainer::tryParsePointData(const std::vector<std::string> &entries) const
{
    if (entries.size() != numberOfObjectPointEntries)
    {
        throw std::invalid_argument("Fatal error. Invalid number of entries for camera data! Data record is missformated!");
    }

    ObjectPoint objectPoint;
    for (const auto [index, entry] : entries | std::views::enumerate | std::views::drop(1) | std::views::take(numberOfObjectPointEntries-1))
    {
        auto value{0.0};
        if (std::from_chars(entry.c_str(), entry.c_str() + entry.length(), value).ec == std::errc()) [[likely]]
        {
            if (std::isnan(value) || std::isinf(value))
            {
                throw std::invalid_argument("Not a number (nan) or inifinite (inf) value detected in camera data");
            }
            if (index < 4)
            {
                objectPoint.point(index-1) = value;

            }
            else
            {
                if (value <= 0)
                {
                    std::cout << "invalid value: " << value <<std::endl;
                    throw std::invalid_argument("Value representing object point uncertainty must be non-negative!");
                }

                objectPoint.uncertainty(index - 4) = value;
            }

        }
        else [[unlikely]]
        {
            throw std::invalid_argument("Not a numeric entry found for object point data, or data record is missformated!");
        }
    }

    PointId pointId{entries[0]};
    objectPoint.id = pointId;
    return std::make_pair(pointId, objectPoint);
}

void ObjectPointContainer::sentToStream(std::ostream &stream) const
{
    for (const auto& [id, pointData]: m_data)
    {
        pointData.sentToStream(stream);
    }
}