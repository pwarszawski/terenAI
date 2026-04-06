// Ten plik zawiera kod na licencji MIT - delaunator.hpp
// 
// Copyright (c) 2018 Volodymyr Bilonenko
// ... (licencja MIT) ...

#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <cmath>
#include <filesystem>
#include <thread>
#include <mutex>
#include <atomic>
#include <algorithm>
#include <iomanip>
#include <unordered_map>
#include <map>
#include <locale>
#include <cstring>
#include <set>
#include <ctime>
#include <cstdint> 

#include "delaunator.hpp" 

namespace fs = std::filesystem;

const std::string PROG_VERSION = "v323.8";

struct GlobalConfig {
    double LimitNMT1 = 150.0;       
    double LimitNMT100Max = 8000.0; 
    double SnapDist = 2.5;          
    double SmoothEnd = 12.0;  
    double TrackOffset = 0.3;       
    
    int NMT1_StepNear = 2;
    double NMT1_DistNear = 15.0;
    int NMT1_StepMid = 5;
    double NMT1_DistMid = 25.0;
    int NMT1_StepFar = 10;

    double MaxTriangleEdge = 800.0; 
    double PlatformExtraOffset = 0.5; 
    
    double EmbankmentWidth = 2.1; 
    double EmbankmentStep = 2.0;   
    double MinPointDist = 1.0;     
    double MergeClosePoints = 0.15; 

    // Konfiguracja wiaduktow
    double BridgeHoleRadius = 5.0; 
    double BridgeClearance = 3.5;  

    std::vector<std::string> DirNMT; 
    std::string FileNMT100 = "NMT100.txt";
    std::vector<std::string> ScnFiles; 
    
    bool ExportSCM = true;
    std::string OutputSCM = "teren.scm";
    bool ExportE3D = false;
    std::string OutputE3D = "teren.e3d";
    double E3DTileSize = 1000.0; 
    double E3DMaxDistance = 50000.0; 

    int CpuUsagePercent = 80;
    int ProgressMode = 2; 
    bool SwapNMT100Axes = true; 
} g_Config;

struct ScnInfo {
    std::string filename;
    double offsetEast = 0.0;
    double offsetNorth = 0.0;
    double shiftX = 0.0;
    double shiftZ = 0.0;
};

struct Vector3 {
    double x, y, z;
    Vector3 operator+(const Vector3& other) const { return {x + other.x, y + other.y, z + other.z}; }
    Vector3 operator-(const Vector3& other) const { return {x - other.x, y - other.y, z - other.z}; }
    Vector3 operator*(double scalar) const { return {x * scalar, y * scalar, z * scalar}; }
    Vector3 operator/(double scalar) const { return {x / scalar, y / scalar, z / scalar}; }
    
    Vector3& operator+=(const Vector3& other) { x += other.x; y += other.y; z += other.z; return *this; }
    double Length() const { return std::sqrt(x*x + y*y + z*z); }
    Vector3 Normalized() const { double len = Length(); if (len < 0.00000001) return {0,1,0}; return {x/len, y/len, z/len}; }
    Vector3 Cross(const Vector3& other) const { return { y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x }; }
};

struct TerrainPoint {
    Vector3 pos;
    Vector3 normal; 
    bool isNMT1;
    bool isValid;
    bool isFixed; 
    
    TerrainPoint() : pos{0,0,0}, normal{0,0,0}, isNMT1(false), isValid(true), isFixed(false) {}
    TerrainPoint(Vector3 p, bool nmt1, bool valid, bool fixed) : pos(p), normal{0,0,0}, isNMT1(nmt1), isValid(valid), isFixed(fixed) {}
};

struct TrackSegment {
    Vector3 p1, p2;
    bool isTunnel;
    bool isBridge;
    bool isPlatform;
    bool isGhost; 
    double minX, maxX, minZ, maxZ; 
    
    TrackSegment() : isTunnel(false), isBridge(false), isPlatform(false), isGhost(false) {}
    void CalcBounds() { minX = std::min(p1.x, p2.x); maxX = std::max(p1.x, p2.x); minZ = std::min(p1.z, p2.z); maxZ = std::max(p1.z, p2.z); }
};

struct TriangleSortInfo {
    size_t v1, v2, v3;
    double cx, cz;
};

// -----------------------------------------------------------
// Klasa dla formatu E3D 
// -----------------------------------------------------------
struct E3DChunkWriter {
    std::vector<uint8_t> data;

    void writeID(const char* id) { data.insert(data.end(), id, id + 4); }
    
    void writeU32(uint32_t val) {
        uint8_t* p = reinterpret_cast<uint8_t*>(&val);
        data.insert(data.end(), p, p + 4);
    }
    
    void writeI32(int32_t val) { writeU32(static_cast<uint32_t>(val)); }
    
    void writeF32(float val) {
        uint8_t* p = reinterpret_cast<uint8_t*>(&val);
        data.insert(data.end(), p, p + 4);
    }
    
    void writeZeroes(size_t count) { data.insert(data.end(), count, 0); }
    
    void pad() { while (data.size() % 4 != 0) data.push_back(0); }
    
    void finalizeLength() {
        if (data.size() >= 8) {
            uint32_t size = static_cast<uint32_t>(data.size());
            std::memcpy(&data[4], &size, 4);
        }
    }
};

std::string Trim(const std::string& str) {
    size_t first = str.find_first_not_of(" \t\r\n");
    if (std::string::npos == first) return str;
    size_t last = str.find_last_not_of(" \t\r\n");
    return str.substr(first, (last - first + 1));
}

void ShowProgress(size_t current, size_t total, const std::string& label) {
    if (g_Config.ProgressMode == 0) return;
    double percent = (double)current / total * 100.0;
    if (percent > 100.0) percent = 100.0;
    if (g_Config.ProgressMode == 1) { std::cout << "\r" << label << ": " << std::fixed << std::setprecision(1) << percent << "% " << std::flush; }
    else if (g_Config.ProgressMode == 2) {
        int barWidth = 30; std::cout << "\r" << label << ": ["; int pos = static_cast<int>(barWidth * percent / 100.0);
        for (int i = 0; i < barWidth; ++i) { if (i < pos) std::cout << "="; else if (i == pos) std::cout << ">"; else std::cout << " "; }
        std::cout << "] " << int(percent) << "% " << std::flush;
    }
}

unsigned int GetThreadCount() {
    unsigned int hw = std::thread::hardware_concurrency(); if (hw == 0) hw = 4;
    int percent = g_Config.CpuUsagePercent; if (percent < 1) percent = 1; if (percent > 100) percent = 100;
    unsigned int target = static_cast<unsigned int>(hw * (percent / 100.0f)); if (target < 1) target = 1; return target;
}

class FastParser {
public:
    static std::vector<char> ReadFileToBuffer(const std::string& path) {
        std::ifstream file(path, std::ios::binary | std::ios::ate); if (!file) return {};
        std::streamsize size = file.tellg(); file.seekg(0, std::ios::beg);
        std::vector<char> buffer(size + 1); if (file.read(buffer.data(), size)) { buffer[size] = '\0'; return buffer; } return {};
    }
};

class MathUtils {
public:
    static double Lerp(double a, double b, double t) { return a + (b - a) * t; }
    
    static Vector3 CubicBezier(double t, Vector3 p1, Vector3 cv1, Vector3 cv2, Vector3 p2) {
        Vector3 c1 = p1 + cv1; 
        Vector3 c2 = p2 + cv2;
        double u = 1.0 - t;
        double tt = t * t, uu = u * u;
        double uuu = uu * u, ttt = tt * t;
        Vector3 p = p1 * uuu;
        p = p + c1 * (3.0 * uu * t);
        p = p + c2 * (3.0 * u * tt);
        p = p + p2 * ttt;
        return p;
    }
    
    static double DistanceSquared2D(const Vector3& a, const Vector3& b) { double dx = a.x - b.x; double dz = a.z - b.z; return dx * dx + dz * dz; }
    static double GetDistanceToSegment(const Vector3& p, const TrackSegment& seg, double& outTrackY) {
        double l2 = DistanceSquared2D(seg.p1, seg.p2);
        if (l2 == 0.0) { outTrackY = seg.p1.y; return std::sqrt(DistanceSquared2D(p, seg.p1)); }
        double t = ((p.x - seg.p1.x) * (seg.p2.x - seg.p1.x) + (p.z - seg.p1.z) * (seg.p2.z - seg.p1.z)) / l2;
        t = std::max(0.0, std::min(1.0, t)); Vector3 proj;
        proj.x = seg.p1.x + t * (seg.p2.x - seg.p1.x); proj.z = seg.p1.z + t * (seg.p2.z - seg.p1.z);
        outTrackY = Lerp(seg.p1.y, seg.p2.y, t); double dx = p.x - proj.x; double dz = p.z - proj.z; return std::sqrt(dx * dx + dz * dz);
    }
    static Vector3 GeoToSim(double geoX_North, double geoY_East, double height, double offsetEastMaster, double offsetNorthMaster) {
        Vector3 v;
        v.x = offsetEastMaster - geoY_East; 
        v.z = geoX_North - offsetNorthMaster;
        v.y = height; 
        return v;
    }
};

template <int CellSize>
class SpatialGrid {
    struct CellKey { int x, z; bool operator==(const CellKey& other) const { return x == other.x && z == other.z; } };
    struct KeyHasher { std::size_t operator()(const CellKey& k) const { return std::hash<int>()(k.x) ^ (std::hash<int>()(k.z) << 1); } };
    std::unordered_map<CellKey, std::vector<const TrackSegment*>, KeyHasher> grid;
    int GetIndex(double val) const { return static_cast<int>(std::floor(val / CellSize)); }
public:
    void Build(const std::vector<TrackSegment>& tracks) {
        grid.clear();
        for (const auto& t : tracks) {
            int minIx = GetIndex(t.minX); int maxIx = GetIndex(t.maxX); int minIz = GetIndex(t.minZ); int maxIz = GetIndex(t.maxZ);
            for (int x = minIx; x <= maxIx; ++x) { for (int z = minIz; z <= maxIz; ++z) { grid[{x, z}].push_back(&t); } }
        }
    }
    void GetNearbyFast(const Vector3& p, std::vector<const TrackSegment*>& out, double maxDist) const {
        out.clear(); int cx = GetIndex(p.x); int cz = GetIndex(p.z); int range = static_cast<int>(std::ceil(maxDist / CellSize));
        if (range < 1) range = 1; 
        for (int x = -range; x <= range; x++) {
            for (int z = -range; z <= range; z++) {
                auto it = grid.find({ cx + x, cz + z });
                if (it != grid.end()) { for(const auto* trk : it->second) out.push_back(trk); }
            }
        }
    }
};

struct LODMap {
    uint8_t* grid = nullptr;
    double minX, minZ;
    int cols, rows;
    double cellSize = 5.0;

    struct StampCell { int dx, dz; uint8_t level; };

    ~LODMap() { if (grid) delete[] grid; }

    void Build(const std::vector<TrackSegment>& tracks) {
        if (tracks.empty()) return;
        double gMinX = 1e9, gMaxX = -1e9, gMinZ = 1e9, gMaxZ = -1e9;
        for (const auto& t : tracks) {
            gMinX = std::min({gMinX, t.minX}); gMaxX = std::max({gMaxX, t.maxX});
            gMinZ = std::min({gMinZ, t.minZ}); gMaxZ = std::max({gMaxZ, t.maxZ});
        }
        
        double margin = g_Config.LimitNMT1 + 20.0;
        minX = gMinX - margin; minZ = gMinZ - margin;
        cols = static_cast<int>((gMaxX - gMinX + 2.0 * margin) / cellSize) + 1;
        rows = static_cast<int>((gMaxZ - gMinZ + 2.0 * margin) / cellSize) + 1;
        
        size_t gridSize = static_cast<size_t>(cols) * rows;
        grid = new uint8_t[gridSize];
        std::memset(grid, 255, gridSize); 
        
        std::vector<StampCell> stamp;
        int searchR = static_cast<int>(std::ceil(g_Config.LimitNMT1 / cellSize)) + 1;
        for (int dx = -searchR; dx <= searchR; ++dx) {
            for (int dz = -searchR; dz <= searchR; ++dz) {
                double dist = std::sqrt(dx*dx + dz*dz) * cellSize;
                if (dist <= g_Config.LimitNMT1) {
                    uint8_t level = 2;
                    if (dist <= g_Config.NMT1_DistNear) level = 0;
                    else if (dist <= g_Config.NMT1_DistMid) level = 1;
                    stamp.push_back({dx, dz, level});
                }
            }
        }

        size_t totalTracks = tracks.size();
        size_t processed = 0;

        for (const auto& t : tracks) {
            processed++;
            if (processed % 500 == 0 || processed == totalTracks) {
                ShowProgress(processed, totalTracks, "[WCZYTYWANIE] Rasteryzacja LOD");
            }
            
            Vector3 dir = t.p2 - t.p1;
            double len = dir.Length();
            int steps = std::max(1, static_cast<int>(len / (cellSize * 0.5)));
            
            for (int i = 0; i <= steps; ++i) {
                Vector3 p = t.p1 + dir * (static_cast<double>(i) / steps);
                int cx = static_cast<int>((p.x - minX) / cellSize);
                int cz = static_cast<int>((p.z - minZ) / cellSize);
                
                for (const auto& s : stamp) {
                    int nx = cx + s.dx;
                    int nz = cz + s.dz;
                    if (nx >= 0 && nx < cols && nz >= 0 && nz < rows) {
                        size_t idx = static_cast<size_t>(nz) * cols + nx;
                        if (s.level < grid[idx]) grid[idx] = s.level;
                    }
                }
            }
        }
        if (g_Config.ProgressMode != 0) std::cout << std::endl;
    }

    uint8_t GetLOD(double x, double z) const {
        if (!grid) return 255;
        int gx = static_cast<int>((x - minX) / cellSize);
        int gz = static_cast<int>((z - minZ) / cellSize);
        if (gx >= 0 && gx < cols && gz >= 0 && gz < rows) {
            return grid[static_cast<size_t>(gz) * cols + gx];
        }
        return 255;
    }
};

void LoadIniConfig(const std::string& filename) {
    if (!fs::exists(filename)) { std::cerr << "BLAD: Brak pliku konfiguracyjnego '.ini'!\n"; exit(1); }
    std::ifstream file(filename); std::string line;
    while (std::getline(file, line)) {
        line = Trim(line); if (line.empty() || line[0] == ';' || line[0] == '#' || line[0] == '[') continue;
        size_t eqPos = line.find('='); if (eqPos == std::string::npos) continue;
        std::string key = Trim(line.substr(0, eqPos)); std::string val = Trim(line.substr(eqPos + 1));
        try {
            if (key == "FileSCN") { 
                std::stringstream ss(val); std::string item; 
                while (std::getline(ss, item, ',')) { g_Config.ScnFiles.push_back(Trim(item)); } 
            }
            else if (key == "DirNMT") { 
                std::stringstream ss(val); std::string item; 
                while (std::getline(ss, item, ',')) { g_Config.DirNMT.push_back(Trim(item)); } 
            }
            else if (key == "FileNMT100") g_Config.FileNMT100 = val;
            else if (key == "ExportSCM") g_Config.ExportSCM = (val == "1" || val == "true" || val == "TRUE");
            else if (key == "OutputSCM") g_Config.OutputSCM = val;
            else if (key == "ExportE3D") g_Config.ExportE3D = (val == "1" || val == "true" || val == "TRUE");
            else if (key == "OutputE3D") g_Config.OutputE3D = val;
            else if (key == "E3DTileSize") g_Config.E3DTileSize = std::stod(val);
            else if (key == "E3DMaxDistance") g_Config.E3DMaxDistance = std::stod(val);
            else if (key == "BridgeHoleRadius") g_Config.BridgeHoleRadius = std::stod(val);
            else if (key == "BridgeClearance") g_Config.BridgeClearance = std::stod(val);
            else if (key == "LimitNMT1") g_Config.LimitNMT1 = std::stod(val);
            else if (key == "LimitNMT100") g_Config.LimitNMT100Max = std::stod(val); 
            else if (key == "SnapDist") g_Config.SnapDist = std::stod(val);
            else if (key == "SmoothDist") g_Config.SmoothEnd = std::stod(val);
            else if (key == "TrackOffset") g_Config.TrackOffset = std::stod(val); 
            else if (key == "NMT1_StepNear") g_Config.NMT1_StepNear = std::stoi(val);
            else if (key == "NMT1_DistNear") g_Config.NMT1_DistNear = std::stod(val);
            else if (key == "NMT1_StepMid") g_Config.NMT1_StepMid = std::stoi(val);
            else if (key == "NMT1_DistMid") g_Config.NMT1_DistMid = std::stod(val);
            else if (key == "NMT1_StepFar") g_Config.NMT1_StepFar = std::stoi(val);
            else if (key == "MaxTriangleEdge") g_Config.MaxTriangleEdge = std::stod(val);
            else if (key == "PlatformExtraOffset") g_Config.PlatformExtraOffset = std::stod(val);
            else if (key == "EmbankmentWidth") g_Config.EmbankmentWidth = std::stod(val); 
            else if (key == "EmbankmentStep") g_Config.EmbankmentStep = std::stod(val);
            else if (key == "MinPointDist") g_Config.MinPointDist = std::stod(val); 
            else if (key == "MergeClosePoints") g_Config.MergeClosePoints = std::stod(val);
            else if (key == "CpuUsagePercent") g_Config.CpuUsagePercent = std::stoi(val); 
            else if (key == "ProgressMode") g_Config.ProgressMode = std::stoi(val);
            else if (key == "SwapNMT100Axes") g_Config.SwapNMT100Axes = (val == "1" || val == "true" || val == "TRUE");
        } catch (...) {}
    }
}

void PrintConfig() {
    std::cout << "\n--------------------------------------" << std::endl;
    std::cout << " Wczytana konfiguracja (z pliku .ini):" << std::endl;
    std::cout << "---------------------------------------" << std::endl;
    std::cout << " [Dane wejsciowe]" << std::endl;
    std::cout << "  Pliki SCN      : " << g_Config.ScnFiles.size() << std::endl;
    std::cout << "  Katalogi NMT1  : " << g_Config.DirNMT.size() << std::endl;
    std::cout << "  Plik NMT100    : " << g_Config.FileNMT100 << std::endl;
    std::cout << "  Wyjscie SCM    : " << (g_Config.ExportSCM ? g_Config.OutputSCM : "Wylaczone") << std::endl;
    std::cout << "  Wyjscie E3D    : " << (g_Config.ExportE3D ? g_Config.OutputE3D + " (Kafle: " + std::to_string((int)g_Config.E3DTileSize) + "m, Widocznosc: " + std::to_string((int)g_Config.E3DMaxDistance) + "m)" : "Wylaczone") << std::endl;
    std::cout << "----------------------------------------\n";
}

bool ReadSingleSCNOffset(const std::string& filename, double& outEast, double& outNorth) {
    std::ifstream file(filename); if (!file.is_open()) return false;
    std::string line;
    for (int i = 0; i < 50; i++) {
        if (!std::getline(file, line)) break;
        if (line.find("//$g") != std::string::npos) {
            std::stringstream ss(line); std::string tag, sys; double kmEast, kmNorth;
            ss >> tag; if (tag == "//$g") { ss >> sys; if (ss >> kmEast >> kmNorth) { outEast = kmEast * 1000.0; outNorth = kmNorth * 1000.0; return true; } }
        }
    } return false;
}

void LoadTracksFromSCN_Global(ScnInfo& sceneInfo, int sceneIdx, std::vector<TrackSegment>& globalTracks) {
    std::ifstream file(sceneInfo.filename);
    if (!file.is_open()) { std::cerr << "BLAD: Nie mozna otworzyc pliku SCN: " << sceneInfo.filename << std::endl; return; }

    std::cout << "[WCZYTYWANIE] tory z pliku: " << sceneInfo.filename;
    if (sceneIdx > 0) std::cout << " (Przesuniecie X=" << std::fixed << std::setprecision(1) << sceneInfo.shiftX << ", Z=" << sceneInfo.shiftZ << ")" << std::endl;
    else std::cout << std::endl;

    std::string line; 
    Vector3 p1, v1, v2, p2; 
    bool foundP1 = false, foundV1 = false, foundV2 = false, foundP2 = false;
    bool inTrack = false, isTunnelTrack = false, isBridgeTrack = false, isPlatformTrack = false;
    
    size_t prevTracksSize = globalTracks.size();

    while (std::getline(file, line)) {
        if (line.empty()) continue;
        
        std::vector<std::string> words; std::stringstream ss(line); std::string word; 
        while (ss >> word) words.push_back(word);
        if (words.empty()) continue;

        if (words[0] == "node" && words.size() >= 6 && words[4] == "track") {
            bool validTrackType = (words[5] == "normal" || words[5] == "switch" || words[5] == "road");
            if (validTrackType) {
                inTrack = true; foundP1 = foundV1 = foundV2 = foundP2 = false; 
                isTunnelTrack = false; isBridgeTrack = false; isPlatformTrack = false;
                
                for (size_t i = 3; i < words.size(); ++i) {
                    std::string w = words[i]; 
                    w.erase(std::remove(w.begin(), w.end(), '\r'), w.end()); 
                    w.erase(std::remove(w.begin(), w.end(), '\n'), w.end());
                    std::transform(w.begin(), w.end(), w.begin(), ::tolower);
                    
                    if (w == "tunnel" || w.find("tunnel") != std::string::npos) isTunnelTrack = true;
                    if (w.find("peron") != std::string::npos) isPlatformTrack = true;
                    if (w == "bridge" || w.find("bridge") != std::string::npos) isBridgeTrack = true;
                }
                continue;
            }
        }

        if (inTrack) {
            std::string lowerLine = line;
            std::transform(lowerLine.begin(), lowerLine.end(), lowerLine.begin(), ::tolower);
            if (lowerLine.find("endtrack") != std::string::npos) { 
                inTrack = false; 
                continue; 
            }

            char fChar = ' '; for(char c : line) { if(!isspace(c)) { fChar = c; break; } }
            if (!isdigit(fChar) && fChar != '-') continue;

            std::stringstream coord_ss(line); double x, y, z; 
            
            if (!foundP1) { 
                if (coord_ss >> x >> y >> z) { p1 = {x + sceneInfo.shiftX, y, z + sceneInfo.shiftZ}; foundP1 = true; } 
            }
            else if (!foundV1) { 
                if (coord_ss >> x >> y >> z) { v1 = {x, y, z}; foundV1 = true; } 
            } 
            else if (!foundV2) { 
                if (coord_ss >> x >> y >> z) { v2 = {x, y, z}; foundV2 = true; } 
            } 
            else if (!foundP2) {
                if (coord_ss >> x >> y >> z) { 
                    p2 = {x + sceneInfo.shiftX, y, z + sceneInfo.shiftZ}; foundP2 = true;

                    Vector3 c1 = p1 + v1;
                    Vector3 c2 = p2 + v2;
                    double polyLen = v1.Length() + (c2 - c1).Length() + v2.Length();
                    
                    int segments = static_cast<int>(polyLen / 2.0);
                    if (segments < 4) segments = 4; 
                    if (segments > 300) segments = 300; 

                    for (int i = 1; i <= segments; i++) {
                        double t = (double)i / (double)segments;
                        Vector3 curr = MathUtils::CubicBezier(t, p1, v1, v2, p2);
                        Vector3 prev = (i == 1) ? p1 : MathUtils::CubicBezier((double)(i - 1) / segments, p1, v1, v2, p2);
                        TrackSegment tSeg; tSeg.p1 = prev; tSeg.p2 = curr; 
                        tSeg.isTunnel = isTunnelTrack; tSeg.isBridge = isBridgeTrack; tSeg.isPlatform = isPlatformTrack;
                        tSeg.CalcBounds(); globalTracks.push_back(tSeg);
                    }
                    
                    foundP1 = false; foundV1 = false; foundV2 = false; foundP2 = false;
                }
            }
        }
    }
    std::cout << " -> Zarejestrowano segmentow toru: " << globalTracks.size() - prevTracksSize << std::endl;
}

void FlagGhostTracks(std::vector<TrackSegment>& tracks) {
    std::cout << "[ANALIZA] Wykrywanie torow na wiaduktach/mostach i tuneli..." << std::endl;
    std::vector<TrackSegment> bridges;
    for (const auto& t : tracks) {
        if (t.isBridge) bridges.push_back(t);
    }
    if (bridges.empty()) return;

    SpatialGrid<150> bridgeGrid; 
    bridgeGrid.Build(bridges);

    int ghosts = 0;
    for (auto& t : tracks) {
        if (t.isBridge || t.isTunnel) continue;
        
        Vector3 mid = (t.p1 + t.p2) * 0.5;
        std::vector<const TrackSegment*> nearbyBridges;
        bridgeGrid.GetNearbyFast(mid, nearbyBridges, 15.0);
        
        for (const auto* b : nearbyBridges) {
            double bY;
            double d = MathUtils::GetDistanceToSegment(mid, *b, bY);
            if (d <= 6.0) { 
                double midY = (t.p1.y + t.p2.y) * 0.5;
                if (std::abs(bY - midY) <= 3.5) { 
                    t.isGhost = true;
                    ghosts++;
                    break;
                }
            }
        }
    }
    std::cout << " -> Zablokowano " << ghosts << " odcinkow torow mostow/wiaduktow i tuneli." << std::endl;
}

void LoadNMT1_DynamicLOD(const std::vector<std::string>& folderPaths, std::vector<TerrainPoint>& outPoints, double masterEast, double masterNorth, const std::vector<TrackSegment>& tracks) {
    std::vector<fs::path> ascFiles;
    std::set<std::string> uniqueFilenames; 

    for (const auto& folderPath : folderPaths) {
        if (!fs::exists(folderPath)) continue;
        for (const auto& entry : fs::directory_iterator(folderPath)) {
            if (entry.path().extension() == ".asc" || entry.path().extension() == ".ASC") {
                std::string fname = entry.path().filename().string();
                if (uniqueFilenames.find(fname) == uniqueFilenames.end()) {
                    uniqueFilenames.insert(fname); ascFiles.push_back(entry.path());
                }
            }
        }
    }
    if (ascFiles.empty()) return;

    LODMap lodMap;
    lodMap.Build(tracks); 

    unsigned int threadsCount = GetThreadCount(); if (threadsCount > 2) threadsCount -= 2; else threadsCount = 1;
    
    std::cout << "[WCZYTYWANIE] Pliki .asc (" << ascFiles.size() << " unikalnych plikow z danymi NMT1)..." << std::endl;
    std::atomic<size_t> filesProcessed(0); std::vector<std::vector<TerrainPoint>> threadBuffers(threadsCount);
    size_t startPointsSize = outPoints.size();

    auto fastParseDouble = [](char** p) -> double {
        char* ptr = *p; while (*ptr && (*ptr == ' ' || *ptr == '\t' || *ptr == '\n' || *ptr == '\r')) ptr++;
        if (!*ptr) { *p = ptr; return 0.0; } bool neg = false;
        if (*ptr == '-') { neg = true; ptr++; } else if (*ptr == '+') { ptr++; }
        double val = 0.0; while (*ptr >= '0' && *ptr <= '9') { val = val * 10.0 + (*ptr - '0'); ptr++; }
        if (*ptr == '.' || *ptr == ',') { ptr++; double frac = 1.0; while (*ptr >= '0' && *ptr <= '9') { frac *= 0.1; val += (*ptr - '0') * frac; ptr++; } }
        if (neg) val = -val; *p = ptr; return val;
    };

    auto fastSkipToken = [](char** p) {
        char* ptr = *p; while (*ptr && (*ptr == ' ' || *ptr == '\t' || *ptr == '\n' || *ptr == '\r')) ptr++;
        while (*ptr && !(*ptr == ' ' || *ptr == '\t' || *ptr == '\n' || *ptr == '\r')) ptr++; *p = ptr;
    };

    auto worker = [&](int threadIdx, size_t startIdx, size_t endIdx) {
        threadBuffers[threadIdx].reserve(500000); 
        for (size_t i = startIdx; i < endIdx; ++i) {
            auto buffer = FastParser::ReadFileToBuffer(ascFiles[i].string()); if (buffer.empty()) { filesProcessed++; continue; }
            char* ptr = buffer.data(); int ncols = 0, nrows = 0; double xll = 0, yll = 0, cellsize = 0, nodata = -9999;
            for(int h=0; h<6; h++) {
                fastSkipToken(&ptr);
                if(h==0) ncols = (int)fastParseDouble(&ptr); else if(h==1) nrows = (int)fastParseDouble(&ptr); else if(h==2) xll = fastParseDouble(&ptr);
                else if(h==3) yll = fastParseDouble(&ptr); else if(h==4) cellsize = fastParseDouble(&ptr); else if(h==5) nodata = fastParseDouble(&ptr);
            }
            if (ncols <= 0 || nrows <= 0 || ncols > 20000 || nrows > 20000) { filesProcessed++; continue; }

            double minGeoX = yll, maxGeoX = yll + nrows * cellsize, minGeoY = xll, maxGeoY = xll + ncols * cellsize;
            Vector3 c1 = MathUtils::GeoToSim(minGeoX, minGeoY, 0, masterEast, masterNorth), c2 = MathUtils::GeoToSim(maxGeoX, maxGeoY, 0, masterEast, masterNorth);
            Vector3 c3 = MathUtils::GeoToSim(minGeoX, maxGeoY, 0, masterEast, masterNorth), c4 = MathUtils::GeoToSim(maxGeoX, minGeoY, 0, masterEast, masterNorth);
            double minSimX = std::min({c1.x, c2.x, c3.x, c4.x}) - g_Config.LimitNMT1, maxSimX = std::max({c1.x, c2.x, c3.x, c4.x}) + g_Config.LimitNMT1;
            double minSimZ = std::min({c1.z, c2.z, c3.z, c4.z}) - g_Config.LimitNMT1, maxSimZ = std::max({c1.z, c2.z, c3.z, c4.z}) + g_Config.LimitNMT1;

            bool isFileNeeded = false;
            for (double x = minSimX; x <= maxSimX; x += 50.0) {
                for (double z = minSimZ; z <= maxSimZ; z += 50.0) { if (lodMap.GetLOD(x, z) < 255) { isFileNeeded = true; break; } }
                if (isFileNeeded) break;
            }
            if (!isFileNeeded) { filesProcessed++; continue; }
            
            for (int r = 0; r < nrows; r++) {
                double currentGeoX = (yll + nrows * cellsize) - (r * cellsize);
                for (int c = 0; c < ncols; c++) {
                    bool isNearNode = (r % g_Config.NMT1_StepNear == 0 && c % g_Config.NMT1_StepNear == 0);
                    bool isMidNode  = (r % g_Config.NMT1_StepMid  == 0 && c % g_Config.NMT1_StepMid  == 0);
                    bool isFarNode  = (r % g_Config.NMT1_StepFar  == 0 && c % g_Config.NMT1_StepFar  == 0);
                    if (!isNearNode && !isMidNode && !isFarNode) { fastSkipToken(&ptr); continue; }

                    double zVal = fastParseDouble(&ptr);
                    if (std::abs(zVal - nodata) > 0.1) {
                        Vector3 pos = MathUtils::GeoToSim(currentGeoX, xll + (c * cellsize), zVal, masterEast, masterNorth);
                        uint8_t lod = lodMap.GetLOD(pos.x, pos.z);
                        if (lod != 255) {
                            bool keep = false;
                            if (lod == 0 && isNearNode) keep = true; else if (lod == 1 && isMidNode) keep = true; else if (lod == 2 && isFarNode) keep = true;
                            if (keep) threadBuffers[threadIdx].push_back({pos, true, true, false});
                        }
                    }
                }
            }
            filesProcessed++;
        }
    };

    std::vector<std::thread> workers; size_t batchSize = (ascFiles.size() + threadsCount - 1) / threadsCount;
    for (unsigned int t = 0; t < threadsCount; ++t) {
        size_t start = t * batchSize; size_t end = std::min(start + batchSize, ascFiles.size());
        if (start < end) workers.emplace_back(worker, t, start, end);
    }
    
    while (filesProcessed < ascFiles.size()) { ShowProgress(filesProcessed, ascFiles.size(), "[WCZYTYWANIE] Wczytywanie plikow z danymi NMT1"); std::this_thread::sleep_for(std::chrono::milliseconds(200)); }
    if (g_Config.ProgressMode != 0) std::cout << std::endl; 
    for (auto& w : workers) w.join();
    for (auto& buf : threadBuffers) outPoints.insert(outPoints.end(), buf.begin(), buf.end());
    
    std::cout << " -> Zaladowano " << outPoints.size() - startPointsSize << " punktow NMT1 z siatki LOD." << std::endl;
}

void LoadNMT100_TXT(const std::string& filename, std::vector<TerrainPoint>& points, double masterEast, double masterNorth) {
    if (!fs::exists(filename)) return;
    std::cout << "[WCZYTYWANIE] Plik NMT100 (" << filename << ")..." << std::endl;
    std::ifstream file(filename); if (!file.is_open()) return; double gX, gY, gH; 
    while (file >> gY >> gX >> gH) { 
        Vector3 pos; if (g_Config.SwapNMT100Axes) pos = MathUtils::GeoToSim(gX, gY, gH, masterEast, masterNorth); else pos = MathUtils::GeoToSim(gY, gX, gH, masterEast, masterNorth);
        points.push_back({pos, false, true, false});
    }
}

void ProcessAndFilterTerrainInRAM(std::vector<TerrainPoint>& points, const std::vector<TrackSegment>& tracks) {
    std::cout << "[FILTROWANIE] Budowa siatek przestrzennych dla torow..." << std::endl;
    SpatialGrid<150> nearGrid; nearGrid.Build(tracks); 
    SpatialGrid<4000> farGrid; farGrid.Build(tracks); 

    std::cout << "[FILTROWANIE] Weryfikacja dystansowa i wygladzanie (" << points.size() << " punktow)..." << std::endl;
    const double LIMIT_NMT1 = g_Config.LimitNMT1;
    const double LIMIT_NMT100 = g_Config.LimitNMT100Max;
    const double SNAP_DIST = g_Config.SnapDist;
    const double SMOOTH_END = g_Config.SmoothEnd;

    unsigned int threads = GetThreadCount(); if (threads > 1) threads -= 1;
    std::vector<std::thread> workers; std::atomic<size_t> processed(0);
    size_t total = points.size(); size_t chunk = (total + threads - 1) / threads;

    auto workerTask = [&](size_t start, size_t end) {
        std::vector<const TrackSegment*> nearbyTracks; nearbyTracks.reserve(100);

        for (size_t i = start; i < end; ++i) {
            TerrainPoint& pt = points[i];
            if (pt.isFixed) { processed++; continue; }

            if (pt.isNMT1) nearGrid.GetNearbyFast(pt.pos, nearbyTracks, std::max(LIMIT_NMT1, SMOOTH_END));
            else farGrid.GetNearbyFast(pt.pos, nearbyTracks, LIMIT_NMT100);

            double distToAnyTrack = 1e9;
            double minTrackDist = 1e9;
            double sumWeightedHeights = 0.0;
            double sumWeights = 0.0;
            
            double minDistBridge = 1e9;
            const TrackSegment* nearestBridge = nullptr;

            for (const auto* trk : nearbyTracks) {
                double trkY;
                double d = MathUtils::GetDistanceToSegment(pt.pos, *trk, trkY);
                if (d < distToAnyTrack) distToAnyTrack = d;
                
                if (trk->isBridge || trk->isTunnel) {
                    if (d < minDistBridge) { minDistBridge = d; nearestBridge = trk; }
                }

                if (!trk->isBridge && !trk->isTunnel && !trk->isGhost) {
                    if (d < minTrackDist) minTrackDist = d;
                    if (d <= SMOOTH_END) {
                        double currentOffset = g_Config.TrackOffset;
                        if (trk->isPlatform) currentOffset += g_Config.PlatformExtraOffset;
                        double targetH = trkY - currentOffset;
                        double d_clamped = std::max(0.1, d); 
                        double weight = 1.0 / (d_clamped * d_clamped * d_clamped);
                        sumWeightedHeights += targetH * weight;
                        sumWeights += weight;
                    }
                }
            }

            bool killPoint = false;
            if (nearestBridge != nullptr && minDistBridge <= g_Config.BridgeHoleRadius && pt.isNMT1) {
                double bY; MathUtils::GetDistanceToSegment(pt.pos, *nearestBridge, bY);
                if (pt.pos.y > bY - g_Config.BridgeClearance) { killPoint = true; }
            }

            if (killPoint) {
                pt.isValid = false;
                processed++;
                continue;
            }

            if (sumWeights > 0.0000001 && minTrackDist <= SMOOTH_END) {
                double blendedTargetH = sumWeightedHeights / sumWeights;
                double blendFactor = 1.0; 
                if (minTrackDist > SNAP_DIST) {
                    double t = (minTrackDist - SNAP_DIST) / (SMOOTH_END - SNAP_DIST);
                    blendFactor = 1.0 - (t * t * (3.0 - 2.0 * t)); 
                }
                pt.pos.y = MathUtils::Lerp(pt.pos.y, blendedTargetH, blendFactor);
            }

            if (pt.isNMT1) { if (distToAnyTrack <= LIMIT_NMT1) pt.isValid = true; else pt.isValid = false; } 
            else { if (distToAnyTrack > LIMIT_NMT1 && distToAnyTrack <= LIMIT_NMT100) pt.isValid = true; else pt.isValid = false; }
            
            processed++;
        }
    };

    for (unsigned int i = 0; i < threads; ++i) { size_t start = i * chunk; size_t end = std::min(start + chunk, total); workers.emplace_back(workerTask, start, end); }
    while(processed < total) { ShowProgress(processed, total, "[FILTROWANIE] Postep"); std::this_thread::sleep_for(std::chrono::milliseconds(200)); }
    if (g_Config.ProgressMode != 0) std::cout << std::endl; 
    for (auto& w : workers) w.join();
    
    size_t validCount = 0;
    for (size_t i = 0; i < points.size(); ++i) { if (points[i].isValid) points[validCount++] = points[i]; }
    points.resize(validCount);
}

void GenerateEmbankmentPoints(const std::vector<TrackSegment>& tracks, std::vector<TerrainPoint>& points) {
    std::cout << "[GENEROWANIE] Obliczanie fizycznych nasypow pod torami..." << std::endl;
    SpatialGrid<150> grid; grid.Build(tracks);
    std::vector<TerrainPoint> newPoints;
    const double OFFSET_XZ = g_Config.EmbankmentWidth;

    size_t totalTracks = tracks.size(); size_t processed = 0;
    
    for (const auto& t : tracks) {
        processed++; if (processed % 1000 == 0 || processed == totalTracks) ShowProgress(processed, totalTracks, "[GENEROWANIE] Wektoryzacja");
        if (t.isTunnel || t.isBridge || t.isGhost) continue; 
        
        Vector3 dir = t.p2 - t.p1; double len = dir.Length(); if (len < 0.1) continue;
        Vector3 dirNorm = dir.Normalized(); Vector3 perp = {-dirNorm.z, 0.0, dirNorm.x};
        double step = g_Config.EmbankmentStep; double currentOffset = g_Config.TrackOffset;
        if (t.isPlatform) currentOffset += g_Config.PlatformExtraOffset;
        
        for (double d = 0.0; d < len - 0.01; d += step) {
            Vector3 centerPos = t.p1 + dirNorm * d;
            double trackHeight = MathUtils::Lerp(t.p1.y, t.p2.y, d/len);
            double terrainHeight = trackHeight - currentOffset;
            
            Vector3 leftPos = centerPos + perp * OFFSET_XZ; leftPos.y = terrainHeight;
            Vector3 rightPos = centerPos - perp * OFFSET_XZ; rightPos.y = terrainHeight;
            
            auto adjustHeightToMaxOverlap = [&](Vector3& pos) {
                double maxH = pos.y; std::vector<const TrackSegment*> nearTracks; grid.GetNearbyFast(pos, nearTracks, 5.0);
                for(const auto* trk : nearTracks) {
                    if (trk == &t || trk->isBridge || trk->isTunnel || trk->isGhost) continue;
                    double trkY; double dist = MathUtils::GetDistanceToSegment(pos, *trk, trkY);
                    if (dist < OFFSET_XZ) { 
                        double otherOffset = g_Config.TrackOffset; if (trk->isPlatform) otherOffset += g_Config.PlatformExtraOffset;
                        double otherTerrainH = trkY - otherOffset; if (otherTerrainH > maxH) maxH = otherTerrainH; 
                    }
                }
                pos.y = maxH;
            };

            adjustHeightToMaxOverlap(leftPos); adjustHeightToMaxOverlap(rightPos);
            newPoints.push_back({leftPos, true, true, true}); newPoints.push_back({rightPos, true, true, true});
        }
    }
    if (g_Config.ProgressMode != 0) std::cout << std::endl;
    points.insert(points.end(), newPoints.begin(), newPoints.end());
}

void RemoveDuplicates(std::vector<TerrainPoint>& points) {
    std::cout << "[OPTYMALIZACJA] Sortowanie i usuwanie duplikatow (" << points.size() << " punktow)..." << std::endl;
    std::sort(points.begin(), points.end(), [](const TerrainPoint& a, const TerrainPoint& b) {
        if (std::abs(a.pos.x - b.pos.x) > 0.001) return a.pos.x < b.pos.x; return a.pos.z < b.pos.z;
    });

    std::vector<TerrainPoint> clean; clean.reserve(points.size());
    const double MD = g_Config.MergeClosePoints; const double MD_SQ = MD * MD;

    size_t total = points.size();
    for (size_t i = 0; i < total; ++i) {
        bool isDup = false;
        for (auto it = clean.rbegin(); it != clean.rend(); ++it) {
            if (points[i].pos.x - it->pos.x > MD) break; 
            if (std::abs(points[i].pos.z - it->pos.z) <= MD) { if (MathUtils::DistanceSquared2D(points[i].pos, it->pos) <= MD_SQ) { isDup = true; break; } }
        }
        if (!isDup) clean.push_back(points[i]);
        if (i % 200000 == 0) ShowProgress(i, total, "[OPTYMALIZACJA] Skanowanie");
    }
    ShowProgress(total, total, "[OPTYMALIZACJA] Skanowanie");
    if (g_Config.ProgressMode != 0) std::cout << std::endl;
    points = std::move(clean);
}

void TriangulateAndExportSingle(std::vector<TerrainPoint>& points) {
    std::cout << "[GENEROWANIE] Triangulacja Delaunaya..." << std::endl;
    std::vector<double> coords; coords.reserve(points.size() * 2);
    for (const auto& p : points) { coords.push_back(p.pos.x); coords.push_back(p.pos.z); }
    delaunator::Delaunator d(coords);

    const double MAX_EDGE = g_Config.MaxTriangleEdge;
    std::vector<TriangleSortInfo> sortedTris; sortedTris.reserve(d.triangles.size() / 3);

    for (size_t i = 0; i < d.triangles.size(); i += 3) {
        size_t v1 = d.triangles[i], v2 = d.triangles[i+1], v3 = d.triangles[i+2];
        Vector3 p1 = points[v1].pos, p2 = points[v2].pos, p3 = points[v3].pos;
        if (std::sqrt(std::pow(p1.x-p2.x, 2) + std::pow(p1.z-p2.z, 2)) > MAX_EDGE || std::sqrt(std::pow(p2.x-p3.x, 2) + std::pow(p2.z-p3.z, 2)) > MAX_EDGE || std::sqrt(std::pow(p3.x-p1.x, 2) + std::pow(p3.z-p1.z, 2)) > MAX_EDGE) continue;

        Vector3 FaceNormal = (p2 - p1).Cross(p3 - p1); 
        if (FaceNormal.Length() < 0.05) continue;

        points[v1].normal += FaceNormal; points[v2].normal += FaceNormal; points[v3].normal += FaceNormal;
        TriangleSortInfo t; t.v1 = v1; t.v2 = v2; t.v3 = v3; t.cx = (p1.x + p2.x + p3.x) / 3.0; t.cz = (p1.z + p2.z + p3.z) / 3.0; 
        sortedTris.push_back(t);
    }
    for(auto& p : points) p.normal = p.normal.Normalized();

    std::cout << "[GENEROWANIE] Sortowanie Z-bufora..." << std::endl;
    std::sort(sortedTris.begin(), sortedTris.end(), [](const TriangleSortInfo& a, const TriangleSortInfo& b) {
        int za = (int)(a.cz / 200.0), zb = (int)(b.cz / 200.0); if (za != zb) return za < zb; return a.cx < b.cx;
    });

    if (g_Config.ExportSCM) {
        std::cout << "[EKSPORT SCM] Zapisywanie spojnego pliku: " << g_Config.OutputSCM << "..." << std::endl;
        std::ofstream out(g_Config.OutputSCM); out.imbue(std::locale("C")); out << std::fixed << std::setprecision(2);
        out << "// Generated by TerenAI " << PROG_VERSION << "\n\n";

        size_t nodeCounter = 0, written = 0, totalTris = sortedTris.size();
        for (size_t i = 0; i < sortedTris.size(); i += 4) {
            written += 4; if (written % 20000 == 0) ShowProgress(written, totalTris, "[EKSPORT SCM] Postep");
            out << "node -1 0 teren_" << nodeCounter++ << " triangles grass\n";
            size_t batchEnd = std::min(i + 4, sortedTris.size());
            for (size_t k = i; k < batchEnd; k++) {
                const auto& t = sortedTris[k]; size_t idx[] = {t.v1, t.v2, t.v3};
                for (int v = 0; v < 3; v++) {
                    const auto& p = points[idx[v]];
                    out << p.pos.x << " " << p.pos.y << " " << p.pos.z << " " << p.normal.x << " " << p.normal.y << " " << p.normal.z << " " << p.pos.x * 0.04 << " " << p.pos.z * 0.04;
                    if (k == batchEnd - 1 && v == 2) out << "\n"; else out << " end\n";
                }
            }
            out << "endtri\n\n";
        }
        if (g_Config.ProgressMode != 0) std::cout << std::endl;
        out.close();
    }

    if (g_Config.ExportE3D) {
        std::cout << "[EKSPORT E3D] Podzial terenu na kafle " << g_Config.E3DTileSize << "x" << g_Config.E3DTileSize << "m..." << std::endl;
        
        double tileSize = g_Config.E3DTileSize;
        std::map<std::pair<int, int>, std::vector<TriangleSortInfo>> tiles;
        
        for (const auto& t : sortedTris) {
            int tx = static_cast<int>(std::floor(t.cx / tileSize));
            int tz = static_cast<int>(std::floor(t.cz / tileSize));
            tiles[{tx, tz}].push_back(t);
        }

        std::string baseE3D = g_Config.OutputE3D;
        if (baseE3D.length() > 4 && baseE3D.substr(baseE3D.length() - 4) == ".e3d") {
            baseE3D = baseE3D.substr(0, baseE3D.length() - 4);
        }
        
        std::string masterScnPath = baseE3D + "_e3d.scm";
        std::ofstream masterScn(masterScnPath);
        
        auto t_now = std::time(nullptr);
        auto tm_now = *std::localtime(&t_now);
        masterScn << "// Generated by TerenAI " << PROG_VERSION << " | Data: " << std::put_time(&tm_now, "%Y-%m-%d %H:%M:%S") << "\n";
        masterScn << "// Zaladuj plik np. poleceniem \"include " << masterScnPath << " end\"\n\n";

        std::string outDir = "kafle_teren";
        if (!fs::exists(outDir)) {
            fs::create_directories(outDir);
        }

        int tileCounter = 0;
        size_t totalTiles = tiles.size();
        
        float maxDistSq = static_cast<float>(g_Config.E3DMaxDistance * g_Config.E3DMaxDistance);
        
        for (auto& kv : tiles) {
            int tx = kv.first.first;
            int tz = kv.first.second;
            auto& tileTris = kv.second;

            double centerPosX = tx * tileSize + tileSize * 0.5;
            double centerPosZ = tz * tileSize + tileSize * 0.5;

            std::unordered_map<size_t, size_t> globalToLocal;
            std::vector<TerrainPoint> localPoints;

            for (const auto& t : tileTris) {
                size_t idx[3] = {t.v1, t.v2, t.v3};
                for (int v = 0; v < 3; v++) {
                    if (globalToLocal.find(idx[v]) == globalToLocal.end()) {
                        globalToLocal[idx[v]] = localPoints.size();
                        TerrainPoint lp = points[idx[v]];
                        lp.pos.x -= centerPosX;
                        lp.pos.z -= centerPosZ;
                        localPoints.push_back(lp);
                    }
                }
            }

            std::string tileFileName = baseE3D + "_" + std::to_string(tx) + "_" + std::to_string(tz) + ".e3d";
            std::string fullTilePath = outDir + "/" + tileFileName;

            E3DChunkWriter tex0, nam0, vnt0, idx4, sub0, e3d0;
            
            tex0.writeID("TEX0"); tex0.writeU32(0); tex0.data.push_back(0); 
            std::string tName = "grass"; tex0.data.insert(tex0.data.end(), tName.begin(), tName.end()); 
            tex0.data.push_back(0); tex0.pad(); tex0.finalizeLength();
            
            nam0.writeID("NAM0"); nam0.writeU32(0); 
            std::string sName = "teren_" + std::to_string(tx) + "_" + std::to_string(tz); 
            nam0.data.insert(nam0.data.end(), sName.begin(), sName.end()); 
            nam0.data.push_back(0); nam0.pad(); nam0.finalizeLength();
            
            vnt0.writeID("VNT0"); vnt0.writeU32(0);
            for (const auto& p : localPoints) { 
                vnt0.writeF32(static_cast<float>(p.pos.x)); 
                vnt0.writeF32(static_cast<float>(p.pos.y)); 
                vnt0.writeF32(static_cast<float>(p.pos.z)); 
                vnt0.writeF32(static_cast<float>(p.normal.x)); 
                vnt0.writeF32(static_cast<float>(p.normal.y)); 
                vnt0.writeF32(static_cast<float>(p.normal.z)); 
                vnt0.writeF32(static_cast<float>((p.pos.x + centerPosX) * 0.04)); 
                vnt0.writeF32(static_cast<float>((p.pos.z + centerPosZ) * 0.04)); 
            }
            vnt0.pad(); vnt0.finalizeLength();
            
            idx4.writeID("IDX4"); idx4.writeU32(0);
            for (const auto& t : tileTris) { 
                idx4.writeU32(static_cast<uint32_t>(globalToLocal[t.v1])); 
                idx4.writeU32(static_cast<uint32_t>(globalToLocal[t.v2])); 
                idx4.writeU32(static_cast<uint32_t>(globalToLocal[t.v3])); 
            }
            idx4.pad(); idx4.finalizeLength();
            
            sub0.writeID("SUB0"); sub0.writeU32(0); sub0.writeI32(-1); sub0.writeI32(-1); sub0.writeI32(4); sub0.writeI32(0); 
            sub0.writeI32(0); sub0.writeI32(16); sub0.writeI32(-1); sub0.writeI32(static_cast<int32_t>(localPoints.size())); 
            sub0.writeI32(0); sub0.writeI32(1); sub0.writeF32(0.0f); sub0.writeF32(0.0f); sub0.writeF32(0); sub0.writeF32(0); 
            sub0.writeF32(0); sub0.writeF32(0); sub0.writeF32(1.0f); sub0.writeF32(1.0f); sub0.writeF32(1.0f); sub0.writeF32(1.0f); 
            sub0.writeF32(0); sub0.writeF32(0); sub0.writeF32(0); sub0.writeF32(0); sub0.writeF32(0); sub0.writeF32(0); sub0.writeF32(0); 
            sub0.writeF32(0); sub0.writeF32(1.0f); sub0.writeF32(maxDistSq); sub0.writeF32(0.0f); sub0.writeZeroes(32); 
            sub0.writeI32(static_cast<int32_t>(tileTris.size() * 3)); sub0.writeI32(0); sub0.writeF32(1.0f); sub0.writeZeroes(88); 
            sub0.finalizeLength();
            
            e3d0.writeID("E3D0"); 
            e3d0.writeU32(8 + static_cast<uint32_t>(sub0.data.size() + vnt0.data.size() + idx4.data.size() + tex0.data.size() + nam0.data.size())); 
            e3d0.data.insert(e3d0.data.end(), sub0.data.begin(), sub0.data.end()); 
            e3d0.data.insert(e3d0.data.end(), vnt0.data.begin(), vnt0.data.end()); 
            e3d0.data.insert(e3d0.data.end(), idx4.data.begin(), idx4.data.end()); 
            e3d0.data.insert(e3d0.data.end(), tex0.data.begin(), tex0.data.end()); 
            e3d0.data.insert(e3d0.data.end(), nam0.data.begin(), nam0.data.end());
            
            std::ofstream file(fullTilePath, std::ios::binary); 
            file.write(reinterpret_cast<const char*>(e3d0.data.data()), e3d0.data.size()); 
            file.close(); 
            
            masterScn << "node -1 0 " << sName << " model " 
                      << std::fixed << std::setprecision(2) << centerPosX << " 0.0 " << centerPosZ 
                      << " 0.0 katalog/" << outDir << "/" << tileFileName << " none endmodel\n";
            
            tileCounter++;
            if (tileCounter % 10 == 0 || tileCounter == totalTiles) ShowProgress(tileCounter, totalTiles, "[EKSPORT E3D] Zapis kafli");
        }
        if (g_Config.ProgressMode != 0) std::cout << std::endl;
        masterScn.close();
        
        std::cout << " -> Zapisano " << totalTiles << " plikow E3D w katalogu '" << outDir << "' oraz plik zbierajacy: " << masterScnPath << std::endl;
    }
}

void ParseArgs(int argc, char* argv[]) { for (int i = 1; i < argc; ++i) { std::string arg = argv[i]; if (arg == "-help" || arg == "/?") { PrintConfig(); exit(0); } } }

int main(int argc, char* argv[]) {
    std::locale::global(std::locale("C")); std::cout << "--- terenAI " << PROG_VERSION << " --- \n"; 
    LoadIniConfig("terenAI.ini"); ParseArgs(argc, argv); 
    if (g_Config.ScnFiles.empty()) { std::cerr << "BLAD: Nie zdefiniowano zadnych plikow w FileSCN!\n"; return 1; }
    PrintConfig();
    
    std::vector<ScnInfo> scenes(g_Config.ScnFiles.size());
    double masterEast = 0.0, masterNorth = 0.0;

    for (size_t i = 0; i < g_Config.ScnFiles.size(); ++i) {
        scenes[i].filename = g_Config.ScnFiles[i];
        if (!ReadSingleSCNOffset(scenes[i].filename, scenes[i].offsetEast, scenes[i].offsetNorth)) {
            std::cout << "Podaj Offset East (km) dla " << scenes[i].filename << ": "; std::cin >> scenes[i].offsetEast; 
            std::cout << "Podaj Offset North (km) dla " << scenes[i].filename << ": "; std::cin >> scenes[i].offsetNorth; 
            scenes[i].offsetEast *= 1000.0; scenes[i].offsetNorth *= 1000.0;
        }
        if (i == 0) { masterEast = scenes[0].offsetEast; masterNorth = scenes[0].offsetNorth; }
        scenes[i].shiftX = masterEast - scenes[i].offsetEast; scenes[i].shiftZ = scenes[i].offsetNorth - masterNorth;
    }

    std::vector<TrackSegment> globalTracks; 
    std::vector<TerrainPoint> globalPoints;

    for (size_t i = 0; i < scenes.size(); ++i) { LoadTracksFromSCN_Global(scenes[i], i, globalTracks); }
    FlagGhostTracks(globalTracks);
    
    LoadNMT1_DynamicLOD(g_Config.DirNMT, globalPoints, masterEast, masterNorth, globalTracks);
    LoadNMT100_TXT(g_Config.FileNMT100, globalPoints, masterEast, masterNorth); 
    
    if (globalPoints.empty()) { std::cerr << "BLAD: Nie wczytano punktow terenu!" << std::endl; return 1; }

    ProcessAndFilterTerrainInRAM(globalPoints, globalTracks); 
    GenerateEmbankmentPoints(globalTracks, globalPoints); 
    RemoveDuplicates(globalPoints);
    TriangulateAndExportSingle(globalPoints);
    
    std::cout << "\nGotowe. Nacisnij Enter, aby zamknac proces..." << std::endl; std::cin.get(); return 0;
}