#include "CameraParametersContainer.hpp"

#include <charconv>
#include <ranges>
#include <fstream>

#include <iostream>

using namespace multicamera_resection;

void CameraParametersContainer::insert(const CameraParameters& cameraParameters)
{
    m_data.insert({cameraParameters.id, cameraParameters});
}

const CameraParameters& CameraParametersContainer::getCameraParameters(const std::string& id) const
{
    auto it = m_data.find(id);
    if (it != m_data.end())
    {
        return it->second;
    }
    else
    {
        throw std::runtime_error("Image parameters with the given ID not found: " + id);
    }
}

void CameraParametersContainer::readDataFromFile(const std::filesystem::path& pathToFile)
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
        entries.reserve(numberOfImageDataEntries);
        for (std::string entry; std::getline(dataRecord, entry, ',');)
        {
            entries.push_back(entry);
        }
        const auto cameraDataWithId{tryParseImageData(entries)};
        const auto &[id, pose]{cameraDataWithId};
        m_data.insert({id, pose});
    }

}


CameraParametersWitId CameraParametersContainer::tryParseImageData(const std::vector<std::string> &entries)
{
    if (entries.size() != numberOfImageDataEntries)
    {
        throw std::invalid_argument("Fatal error. Invalid number of entries for camera data! Data record is missformated!");
    }

    CameraParameters cameraParameters;

    for (const auto [index, entry] : entries | std::views::enumerate | std::views::drop(1) | std::views::take(numberOfImageDataEntries-1))
    {

        auto value{0.0};
        if (std::from_chars(entry.c_str(), entry.c_str() + entry.length(), value).ec == std::errc()) [[likely]]
        {
            if (std::isnan(value) || std::isinf(value))
            {
                throw std::invalid_argument("Not a number (nan) or inifinite (inf) value detected in camera data");
            }

            if (index < 7)
            {
                if (value < 0.0)
                {
                    std::cout << "invalid value: " << value <<std::endl;
                    throw std::invalid_argument("Negative value detected in camera data for camera matrix coefficient or camera dimension! Values must be non-negative!");
                }
                
                if (index == 1)
                {
                    cameraParameters.cameraMatrix(0,0) = value;
                }

                if (index == 2)
                {
                    cameraParameters.cameraMatrix(1,1) = value;
                }

                if (index == 3)
                {
                    cameraParameters.cameraMatrix(0,2) = value;
                }

                if (index == 4)
                {
                    cameraParameters.cameraMatrix(1,2) = value;
                }

                if (index == 5)
                {
                    cameraParameters.imageWidth = static_cast<int>(value);
                }

                if (index == 6)
                {
                    cameraParameters.imageHeight = static_cast<int>(value);
                }
            }

            if (index >= 7 && index <= 9)
            {
                cameraParameters.position(index - 7) = value;
            }

            if (index >= 10 && index <= 13)
            {
                if (value < -1.0 || value > 1.0)
                {
                    std::cout << "invalid value: " << value <<std::endl;
                    throw std::invalid_argument("Invalid value for quaternion coefficient detected in camera data! Quaternion coefficients must be in range [-1.0, 1.0]");
                }
                
                if (index == 10)
                {
                    cameraParameters.quaternion.w() = value;
                }
                else if (index == 11)
                {
                    cameraParameters.quaternion.x() = value;
                }
                else if (index == 12)
                {
                    cameraParameters.quaternion.y() = value;
                }
                else if (index == 13)
                {
                    cameraParameters.quaternion.z() = value;
                }
            }
            cameraParameters.quaternion.normalize();

        }
        else [[unlikely]]
        {
            throw std::invalid_argument("Not a numeric entry found for camera data, or data record is missformated!");
        }
    }

    CameraId cameraId{entries[0]};
    cameraParameters.id = cameraId;
    return std::make_pair(cameraId, cameraParameters);
}


void CameraParametersContainer::sentToStream(std::ostream &outputStream) const
{
    for (const auto&[id, cameraData] : m_data )
    {
        cameraData.sentToStream(outputStream);
    }
}

size_t CameraParametersContainer::size() const
{
    return m_data.size();
}