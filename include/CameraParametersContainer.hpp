#pragma once

#include <map>
#include <string>
#include <filesystem>
#include "CameraParameters.hpp"

namespace multicamera_resection
{   

using CameraParametersWitId = std::pair<CameraId, CameraParameters>;

static constexpr size_t numberOfImageDataEntries{14uz};

class CameraParametersContainer
{
public:
    void insert(const CameraParameters& imageParameters);
    const CameraParameters& getCameraParameters(const std::string& id) const;
    void readDataFromFile(const std::filesystem::path& pathToFile);
    void sentToStream(std::ostream &outputStream) const;

    size_t size() const;

    const std::map<CameraId, CameraParameters>& getData() const
    {
        return m_data;
    }

    std::map<CameraId, CameraParameters>& getData()
    {
        return m_data;
    }


private:
    std::map<CameraId, CameraParameters> m_data;
    CameraParametersWitId tryParseImageData(const std::vector<std::string> &entries);

};

}