#include <Windows.h>

#include "E57Simple.h"

#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/filereadstream.h"   // FileReadStream
#include "rapidjson/filewritestream.h"
#include "rapidjson/encodedstream.h"    // AutoUTFInputStream
#include "rapidjson/writer.h"

// E57 Pose Extractor
// dependencies: libe57 (simple API) and rapidjson

using namespace rapidjson;

int main(int argc, char *argv[]) {
    //
    Document json(kObjectType);
    //
    //
    const std::string FN = "I:\\Projects\\2017-07-GIMPublication\\FojabVillaPierce_1stFloor\\FojabVillaPierce_1Floor.e57";
    e57::Reader     eReader(FN.c_str());
    //
    e57::E57Root    rootHeader;
    eReader.GetE57Root(rootHeader);
    //
    Value idvalue(rootHeader.guid.c_str(), json.GetAllocator());
    json.AddMember("id", idvalue, json.GetAllocator());
    Value fnvalue(FN.c_str(), json.GetAllocator());
    json.AddMember("file", fnvalue, json.GetAllocator());
    //
    std::cout << "guid: " << rootHeader.guid.c_str() << std::endl;
    //
    const int numScans = eReader.GetData3DCount();
    std::cout << "number of scans: " << numScans << std::endl;
    //
    Value scans(kArrayType);

    for (int i = 0; i < numScans; ++i) {
        Value scan(kObjectType);;
        //
        e57::Data3D scanHeader;
        eReader.ReadData3D(i, scanHeader);
        //
        std::cout << "SCAN " << i << std::endl;
        //
        Value valName(scanHeader.name.c_str(), json.GetAllocator());
        scan.AddMember("name", valName, json.GetAllocator());
        std::cout << " Name:" << scanHeader.name << std::endl;
        //
        Value pos(kObjectType);
        auto const &p = scanHeader.pose.translation;
        pos.AddMember("x", p.x, json.GetAllocator());
        pos.AddMember("y", p.y, json.GetAllocator());
        pos.AddMember("z", p.z, json.GetAllocator());
        scan.AddMember("pos", pos, json.GetAllocator());
        std::cout << " pos:  x:" << p.x << " y:" << p.y << " z:" << p.z << std::endl;
        //
        Value rot(kObjectType);
        auto const &q = scanHeader.pose.rotation;
        rot.AddMember("x", q.x, json.GetAllocator());
        rot.AddMember("y", q.y, json.GetAllocator());
        rot.AddMember("z", q.z, json.GetAllocator());
        rot.AddMember("w", q.w, json.GetAllocator());
        scan.AddMember("rot", rot, json.GetAllocator());
        std::cout << " rot:  w:" << q.w << " x:" << q.x << " y:" << q.y << " z:" << q.z << std::endl;
        //
        Value bounds(kObjectType);
        bounds.AddMember("elevation_minimum", scanHeader.sphericalBounds.elevationMinimum, json.GetAllocator());
        bounds.AddMember("elevation_maximum", scanHeader.sphericalBounds.elevationMaximum, json.GetAllocator());
        bounds.AddMember("azimuth_start", scanHeader.sphericalBounds.azimuthStart, json.GetAllocator());
        bounds.AddMember("azimuth_end", scanHeader.sphericalBounds.azimuthEnd, json.GetAllocator());
        scan.AddMember("bounds", bounds, json.GetAllocator());
        scans.PushBack(scan, json.GetAllocator());
    }
    //
    Value scanskey("scans", json.GetAllocator());
    json.AddMember(scanskey, scans, json.GetAllocator());
    //
    {
        FILE* fp = fopen("output.json", "wb"); // non-Windows use "w"
        char writeBuffer[65536];
        FileWriteStream os(fp, writeBuffer, sizeof(writeBuffer));
        Writer<FileWriteStream> writer(os);
        json.Accept(writer);
        fclose(fp);
    }
}

