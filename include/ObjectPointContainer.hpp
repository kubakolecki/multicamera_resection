#pragma once

#include "ObjectPoint.hpp"

#include <map>
#include <string>
#include <filesystem>

namespace multicamera_resection
{

using ObjectPointWitId = std::pair<CameraId, ObjectPoint>;    

class ObjectPointContainer
{
    public:
        

        void readDataFromFile(const std::filesystem::path& pathToFile);

        const std::map<CameraId, ObjectPoint>& getData() const
        {
            return m_data;
        }

        std::map<CameraId, ObjectPoint>& getData()
        {
            return m_data;
        }

        void sentToStream(std::ostream &stream) const;

    private:
        static constexpr size_t numberOfObjectPointEntries{7uz};
        std::map<PointId, ObjectPoint> m_data;
        ObjectPointWitId tryParsePointData(const std::vector<std::string> &entries) const; 


};

}
