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

// -----------------------------------------------------------
// Plik delaunator.hpp musi być w katalogu projektu
// -----------------------------------------------------------
#include "delaunator.hpp" 

namespace fs = std::filesystem;

// -----------------------------------------------------------
// Konfiguracja i struktury danych
// -----------------------------------------------------------

const std::string PROG_VERSION = "v283";

struct GlobalConfig {
    double OffsetEast = 0.0;
    double OffsetNorth = 0.0;
    bool OffsetsLoaded = false;
    
    // Parametry przetwarzania terenu
    float LimitNMT1 = 150.0f;       
    float LimitNMT100Max = 8000.0f; 
    float SnapDist = 4.5f;          
    float SmoothEnd = 15.0f;        
    float MaxTriangleEdge = 800.0f; 
    float TrackOffset = 0.6f; 
    float PlatformExtraOffset = 0.5f; 
    
    // Parametry nasypów
    float EmbankmentWidth = 2.60f; 
    float EmbankmentStep = 5.0f;   
    float MinPointDist = 1.0f;     
    float MergeClosePoints = 0.15f; 

    // Parametry 3-stopniowego Downsamplingu NMT1
    int NMT1_StepNear = 2;        
    float NMT1_DistNear = 40.0f;  
    int NMT1_StepMid = 5;         
    float NMT1_DistMid = 120.0f;  
    int NMT1_StepFar = 10;        

    // Nazwy plików i katalogów
    std::string DirNMT1 = "NMT1";
    std::string FileNMT100 = "NMT100.txt";
    std::vector<std::string> ScnFiles; 
    
    // Ustawienia Eksportu (z pliku .ini)
    bool ExportSCM = true;
    std::string OutputSCM = "teren.scm";
    bool ExportE3D = false;
    std::string OutputE3D = "teren.e3d";

    int CpuUsagePercent = 80;
    int ProgressMode = 2; 
    bool SwapNMT100Axes = true; 
} g_Config;

struct Vector3 {
    float x, y, z;
    Vector3 operator+(const Vector3& other) const { return {x + other.x, y + other.y, z + other.z}; }
    Vector3 operator-(const Vector3& other) const { return {x - other.x, y - other.y, z - other.z}; }
    Vector3 operator*(float scalar) const { return {x * scalar, y * scalar, z * scalar}; }
    Vector3 operator/(float scalar) const { return {x / scalar, y / scalar, z / scalar}; }
    
    Vector3& operator+=(const Vector3& other) {
        x += other.x; y += other.y; z += other.z;
        return *this;
    }

    float Length() const { return std::sqrt(x*x + y*y + z*z); }
    
    Vector3 Normalized() const {
        float len = Length();
        if (len < 0.00001f) return {0,1,0}; 
        return {x/len, y/len, z/len};
    }

    Vector3 Cross(const Vector3& other) const {
        return {
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        };
    }
};

struct TerrainPoint {
    Vector3 pos;
    Vector3 normal; 
    bool isNMT1;
    bool isValid;
    bool isFixed; 
    
    TerrainPoint() : pos{0,0,0}, normal{0,0,0}, isNMT1(false), isValid(true), isFixed(false) {}
    TerrainPoint(Vector3 p, bool nmt1, bool valid, bool fixed) 
        : pos(p), normal{0,0,0}, isNMT1(nmt1), isValid(valid), isFixed(fixed) {}
};

struct TrackSegment {
    Vector3 p1, p2;
    bool isTunnel;
    bool isBridge;
    bool isPlatform;
    bool isGhost;
    float minX, maxX, minZ, maxZ; 
    
    TrackSegment() : isTunnel(false), isBridge(false), isPlatform(false), isGhost(false) {}
    
    void CalcBounds() {
        minX = std::min(p1.x, p2.x); maxX = std::max(p1.x, p2.x);
        minZ = std::min(p1.z, p2.z); maxZ = std::max(p1.z, p2.z);
    }
};

struct TriangleSortInfo {
    size_t v1, v2, v3;
    float cx, cz;
};

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
    float percent = (float)current / total * 100.0f;
    if (percent > 100.0f) percent = 100.0f;
    if (g_Config.ProgressMode == 1) {
        std::cout << "\r" << label << ": " << std::fixed << std::setprecision(1) << percent << "% " << std::flush;
    }
    else if (g_Config.ProgressMode == 2) {
        int barWidth = 30;
        std::cout << "\r" << label << ": [";
        int pos = barWidth * percent / 100.0f;
        for (int i = 0; i < barWidth; ++i) {
            if (i < pos) std::cout << "=";
            else if (i == pos) std::cout << ">";
            else std::cout << " ";
        }
        std::cout << "] " << int(percent) << "% " << std::flush;
    }
}

unsigned int GetThreadCount() {
    unsigned int hw = std::thread::hardware_concurrency();
    if (hw == 0) hw = 4;
    int percent = g_Config.CpuUsagePercent;
    if (percent < 1) percent = 1; if (percent > 100) percent = 100;
    unsigned int target = static_cast<unsigned int>(hw * (percent / 100.0f));
    if (target < 1) target = 1;
    return target;
}

class FastParser {
public:
    static std::vector<char> ReadFileToBuffer(const std::string& path) {
        std::ifstream file(path, std::ios::binary | std::ios::ate);
        if (!file) return {};
        std::streamsize size = file.tellg();
        file.seekg(0, std::ios::beg);
        std::vector<char> buffer(size + 1);
        if (file.read(buffer.data(), size)) { buffer[size] = '\0'; return buffer; }
        return {};
    }
    static char* SkipWhitespace(char* ptr) {
        while (*ptr && (*ptr == ' ' || *ptr == '\t' || *ptr == '\n' || *ptr == '\r')) ptr++;
        return ptr;
    }
};

class MathUtils {
public:
    static float Lerp(float a, float b, float t) { return a + (b - a) * t; }
    static Vector3 CubicBezier(float t, Vector3 p1, Vector3 cv1, Vector3 cv2, Vector3 p2) {
        Vector3 c1 = p1 + cv1; Vector3 c2 = p2 + cv2;
        float u = 1.0f - t;
        float tt = t * t, uu = u * u, uuu = uu * u, ttt = tt * t;
        Vector3 p = p1 * uuu; p = p + c1 * (3 * uu * t); p = p + c2 * (3 * u * tt); p = p + p2 * ttt;
        return p;
    }
    static float DistanceSquared2D(const Vector3& a, const Vector3& b) {
        float dx = a.x - b.x; float dz = a.z - b.z; return dx * dx + dz * dz;
    }
    static float GetDistanceToSegment(const Vector3& p, const TrackSegment& seg, float& outTrackY) {
        float l2 = DistanceSquared2D(seg.p1, seg.p2);
        if (l2 == 0.0f) { outTrackY = seg.p1.y; return std::sqrt(DistanceSquared2D(p, seg.p1)); }
        float t = ((p.x - seg.p1.x) * (seg.p2.x - seg.p1.x) + (p.z - seg.p1.z) * (seg.p2.z - seg.p1.z)) / l2;
        t = std::max(0.0f, std::min(1.0f, t));
        Vector3 proj;
        proj.x = seg.p1.x + t * (seg.p2.x - seg.p1.x); proj.z = seg.p1.z + t * (seg.p2.z - seg.p1.z);
        outTrackY = Lerp(seg.p1.y, seg.p2.y, t);
        float dx = p.x - proj.x; float dz = p.z - proj.z;
        return std::sqrt(dx * dx + dz * dz);
    }
    static Vector3 GeoToSim(double geoX_North, double geoY_East, float height) {
        Vector3 v;
        v.x = static_cast<float>(g_Config.OffsetEast - geoY_East); 
        v.z = static_cast<float>(geoX_North - g_Config.OffsetNorth);
        v.y = height; return v;
    }
};

template <int CellSize>
class SpatialGrid {
    struct CellKey { int x, z; bool operator==(const CellKey& other) const { return x == other.x && z == other.z; } };
    struct KeyHasher { std::size_t operator()(const CellKey& k) const { return std::hash<int>()(k.x) ^ (std::hash<int>()(k.z) << 1); } };
    std::unordered_map<CellKey, std::vector<const TrackSegment*>, KeyHasher> grid;
    int GetIndex(float val) const { return static_cast<int>(std::floor(val / CellSize)); }
public:
    void Build(const std::vector<TrackSegment>& tracks) {
        grid.clear();
        for (const auto& t : tracks) {
            int minIx = GetIndex(t.minX); int maxIx = GetIndex(t.maxX);
            int minIz = GetIndex(t.minZ); int maxIz = GetIndex(t.maxZ);
            for (int x = minIx; x <= maxIx; ++x) {
                for (int z = minIz; z <= maxIz; ++z) { grid[{x, z}].push_back(&t); }
            }
        }
    }
    void GetNearby(const Vector3& p, std::vector<const TrackSegment*>& out) const {
        out.clear();
        int cx = GetIndex(p.x); int cz = GetIndex(p.z);
        int range = (CellSize > 1000) ? 4 : 3;
        for (int x = -range; x <= range; x++) {
            for (int z = -range; z <= range; z++) {
                CellKey k = { cx + x, cz + z };
                auto it = grid.find(k);
                if (it != grid.end()) out.insert(out.end(), it->second.begin(), it->second.end());
            }
        }
        if (!out.empty()) {
            std::sort(out.begin(), out.end());
            auto last = std::unique(out.begin(), out.end());
            out.erase(last, out.end());
        }
    }
};

class PointGrid {
    struct CellKey { int x, z; bool operator==(const CellKey& other) const { return x == other.x && z == other.z; } };
    struct KeyHasher { std::size_t operator()(const CellKey& k) const { return std::hash<int>()(k.x) ^ (std::hash<int>()(k.z) << 1); } };
    std::unordered_map<CellKey, std::vector<Vector3>, KeyHasher> grid;
    const int CellSize = 10; 
    int GetIndex(float val) const { return static_cast<int>(std::floor(val / CellSize)); }
public:
    void Add(const Vector3& p) { grid[{GetIndex(p.x), GetIndex(p.z)}].push_back(p); }
    bool HasNeighbor(const Vector3& p, float radius) const {
        int cx = GetIndex(p.x); int cz = GetIndex(p.z); float r2 = radius * radius;
        for (int x = -1; x <= 1; x++) {
            for (int z = -1; z <= 1; z++) {
                auto it = grid.find({cx + x, cz + z});
                if (it != grid.end()) {
                    for (const auto& other : it->second) {
                        if (MathUtils::DistanceSquared2D(p, other) < r2) return true;
                    }
                }
            }
        }
        return false;
    }
};

void LoadIniConfig(const std::string& filename) {
    if (!fs::exists(filename)) { std::cerr << "BLAD: Brak pliku konfiguracyjnego '.ini'!\n"; exit(1); }
    std::ifstream file(filename); std::string line;
    while (std::getline(file, line)) {
        line = Trim(line);
        if (line.empty() || line[0] == ';' || line[0] == '#' || line[0] == '[') continue;
        size_t eqPos = line.find('='); if (eqPos == std::string::npos) continue;
        std::string key = Trim(line.substr(0, eqPos)); std::string val = Trim(line.substr(eqPos + 1));
        try {
            if (key == "FileSCN") {
                std::stringstream ss(val); std::string item;
                while (std::getline(ss, item, ',')) { g_Config.ScnFiles.push_back(Trim(item)); }
            }
            else if (key == "DirNMT1") g_Config.DirNMT1 = val; else if (key == "FileNMT100") g_Config.FileNMT100 = val;
            else if (key == "ExportSCM") g_Config.ExportSCM = (val == "1" || val == "true" || val == "TRUE");
            else if (key == "OutputSCM" || key == "Output") g_Config.OutputSCM = val;
            else if (key == "ExportE3D") g_Config.ExportE3D = (val == "1" || val == "true" || val == "TRUE");
            else if (key == "OutputE3D") g_Config.OutputE3D = val;
            else if (key == "NMT1_StepNear") g_Config.NMT1_StepNear = std::stoi(val); else if (key == "NMT1_DistNear") g_Config.NMT1_DistNear = std::stof(val);
            else if (key == "NMT1_StepMid") g_Config.NMT1_StepMid = std::stoi(val); else if (key == "NMT1_DistMid") g_Config.NMT1_DistMid = std::stof(val);
            else if (key == "NMT1_StepFar") g_Config.NMT1_StepFar = std::stoi(val); else if (key == "LimitNMT1") g_Config.LimitNMT1 = std::stof(val);
            else if (key == "LimitNMT100") g_Config.LimitNMT100Max = std::stof(val); else if (key == "SnapDist") g_Config.SnapDist = std::stof(val);
            else if (key == "SmoothDist") g_Config.SmoothEnd = std::stof(val); else if (key == "MaxTriangleEdge") g_Config.MaxTriangleEdge = std::stof(val);
            else if (key == "TrackOffset") g_Config.TrackOffset = std::stof(val); else if (key == "PlatformExtraOffset") g_Config.PlatformExtraOffset = std::stof(val);
            else if (key == "EmbankmentWidth") g_Config.EmbankmentWidth = std::stof(val); else if (key == "EmbankmentStep") g_Config.EmbankmentStep = std::stof(val);
            else if (key == "MinPointDist") g_Config.MinPointDist = std::stof(val); else if (key == "MergeClosePoints") g_Config.MergeClosePoints = std::stof(val);
            else if (key == "CpuUsagePercent") g_Config.CpuUsagePercent = std::stoi(val); else if (key == "ProgressMode") g_Config.ProgressMode = std::stoi(val);
            else if (key == "SwapNMT100Axes") g_Config.SwapNMT100Axes = (val == "1" || val == "true" || val == "TRUE");
        } catch (...) {}
    }
    if (g_Config.NMT1_StepNear < 1) g_Config.NMT1_StepNear = 1; if (g_Config.NMT1_StepMid < 1) g_Config.NMT1_StepMid = 1; if (g_Config.NMT1_StepFar < 1) g_Config.NMT1_StepFar = 1;
}

void PrintConfig() {
    std::cout << "\n--------------------------------------" << std::endl;
    std::cout << " Wczytana konfiguracja (z pliku .ini):" << std::endl;
    std::cout << "---------------------------------------" << std::endl;
    std::cout << " [WEJSCIE]" << std::endl;
    std::cout << "  Pliki torow SCN : ";
    for (const auto& f : g_Config.ScnFiles) std::cout << f << " ";
    std::cout << "\n  Katalog NMT1   : " << g_Config.DirNMT1 << std::endl;
    std::cout << "  Plik NMT100    : " << g_Config.FileNMT100 << std::endl;
    std::cout << "\n [DOWNSAMPLING LOD NMT1]" << std::endl;
    std::cout << "  0 - " << g_Config.NMT1_DistNear << "m : Krok " << g_Config.NMT1_StepNear << "m" << std::endl;
    std::cout << "  " << g_Config.NMT1_DistNear << " - " << g_Config.NMT1_DistMid << "m : Krok " << g_Config.NMT1_StepMid << "m" << std::endl;
    std::cout << "  > " << g_Config.NMT1_DistMid << "m : Krok " << g_Config.NMT1_StepFar << "m" << std::endl;
    std::cout << "----------------------------------------\n\n";
}

bool ParseSCNOffsets(const std::vector<std::string>& filenames) {
    for (const auto& filename : filenames) {
        std::ifstream file(filename); if (!file.is_open()) continue;
        std::string line;
        for (int i = 0; i < 50; i++) {
            if (!std::getline(file, line)) break;
            if (line.find("//$g") != std::string::npos) {
                std::stringstream ss(line); std::string tag, sys; double kmEast, kmNorth;
                ss >> tag; 
                if (tag == "//$g") { ss >> sys; if (ss >> kmEast >> kmNorth) { g_Config.OffsetEast = kmEast * 1000.0; g_Config.OffsetNorth = kmNorth * 1000.0; g_Config.OffsetsLoaded = true; return true; } }
            }
        }
    }
    return false;
}

void LoadTracksFromSCN(const std::string& filename, std::vector<TrackSegment>& tracks) {
    std::ifstream file(filename);
    if (!file.is_open()) { std::cerr << "BLAD: Nie mozna otworzyc pliku SCN: " << filename << std::endl; return; }
    std::string line; Vector3 p1, cv1, cv2, p2;
    bool foundP1 = false, foundCV1 = false, foundCV2 = false, foundP2 = false;
    bool inTrack = false, isTunnelTrack = false, isBridgeTrack = false, isPlatformTrack = false;
    int totalSegments = 0;

    std::cout << "[WCZYTYWANIE] tory z pliku: " << filename << " ..." << std::endl;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::vector<std::string> words; std::stringstream ss(line); std::string word;
        while (ss >> word) words.push_back(word);

        if (words.size() >= 5 && words[0] == "node" && words[4] == "track") {
            bool validTrackType = (words[5] == "normal" || words[5] == "switch" || words[5] == "road");
            if (validTrackType) {
                inTrack = true; foundP1 = foundCV1 = foundCV2 = foundP2 = false; 
                isTunnelTrack = false; isBridgeTrack = false; isPlatformTrack = false;

                for (size_t i = 3; i < words.size(); ++i) {
                    std::string w = words[i];
                    w.erase(std::remove(w.begin(), w.end(), '\r'), w.end());
                    w.erase(std::remove(w.begin(), w.end(), '\n'), w.end());
                    std::transform(w.begin(), w.end(), w.begin(), ::tolower);
                    if (w == "bridge" || w.find("bridge") != std::string::npos) isBridgeTrack = true;
                    if (w == "tunnel" || w.find("tunnel") != std::string::npos) isTunnelTrack = true;
                    if (w.find("peron") != std::string::npos) isPlatformTrack = true;
                }
                continue;
            }
        }

        if (inTrack) {
            if (line.find("endtrack") != std::string::npos || line.find("ENDTRACK") != std::string::npos) { inTrack = false; continue; }
            char firstChar = ' '; for(char c : line) { if(!isspace(c)) { firstChar = c; break; } }
            if (!isdigit(firstChar) && firstChar != '-') continue;

            std::stringstream coord_ss(line); float x, y, z; 
            if (!foundP1) { if (coord_ss >> x >> y >> z) { p1 = {x, y, z}; foundP1 = true; } }
            else if (!foundCV1) { if (coord_ss >> x >> y >> z) { cv1 = {x, y, z}; foundCV1 = true; } }
            else if (!foundCV2) { if (coord_ss >> x >> y >> z) { cv2 = {x, y, z}; foundCV2 = true; } }
            else if (!foundP2) {
                if (coord_ss >> x >> y >> z) { 
                    p2 = {x, y, z}; foundP2 = true;
                    float approxLen = std::sqrt(std::pow(p1.x-p2.x, 2) + std::pow(p1.y-p2.y, 2) + std::pow(p1.z-p2.z, 2));
                    int segments = static_cast<int>(approxLen / 5.0f);
                    if (segments < 2) segments = 2; if (segments > 48) segments = 48;

                    for (int i = 1; i <= segments; i++) {
                        float t = (float)i / (float)segments;
                        Vector3 curr = MathUtils::CubicBezier(t, p1, cv1, cv2, p2);
                        Vector3 prev = (i == 1) ? p1 : MathUtils::CubicBezier((float)(i - 1) / segments, p1, cv1, cv2, p2);
                        TrackSegment tSeg; tSeg.p1 = prev; tSeg.p2 = curr; 
                        tSeg.isTunnel = isTunnelTrack; tSeg.isBridge = isBridgeTrack; tSeg.isPlatform = isPlatformTrack;
                        tSeg.isGhost = false;
                        tSeg.CalcBounds(); tracks.push_back(tSeg); totalSegments++;
                    }
                }
            }
        }
    }
    std::cout << " -> " << totalSegments << " segmentow." << std::endl;
}

void FlagGhostTracks(std::vector<TrackSegment>& tracks) {
    std::cout << "[ANALIZA] Wykrywanie torow na wiaduktach..." << std::endl;
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
        
        Vector3 mid = (t.p1 + t.p2) * 0.5f;
        std::vector<const TrackSegment*> nearbyBridges;
        bridgeGrid.GetNearby(mid, nearbyBridges);
        
        for (const auto* b : nearbyBridges) {
            float bY;
            float d = MathUtils::GetDistanceToSegment(mid, *b, bY);
            if (d <= 6.0f) { 
                float midY = (t.p1.y + t.p2.y) * 0.5f;
                if (std::abs(bY - midY) <= 3.5f) { 
                    t.isGhost = true;
                    ghosts++;
                    break;
                }
            }
        }
    }
    std::cout << " -> Oznaczono " << ghosts << " segmentow jako tory na wiaduktach(zostana zignorowane)." << std::endl;
}

void LoadNMT1_Turbo(const std::string& folderPath, std::vector<TerrainPoint>& outPoints, const std::vector<TrackSegment>& tracks) {
    if (!fs::exists(folderPath)) return;
    std::vector<fs::path> ascFiles;
    for (const auto& entry : fs::directory_iterator(folderPath)) { if (entry.path().extension() == ".asc") ascFiles.push_back(entry.path()); }
    if (ascFiles.empty()) return;

    SpatialGrid<200> trackGrid; trackGrid.Build(tracks);
    unsigned int threadsCount = GetThreadCount(); if (threadsCount > 2) threadsCount -= 2; else threadsCount = 1;

    std::cout << "[WCZYTYWANIE] NMT1 z pikselowym LOD (" << ascFiles.size() << " plikow)..." << std::endl;
    std::atomic<size_t> filesProcessed(0); std::vector<std::vector<TerrainPoint>> threadBuffers(threadsCount);

    auto worker = [&](int threadIdx, size_t startIdx, size_t endIdx) {
        threadBuffers[threadIdx].reserve(1000000); 
        std::vector<const TrackSegment*> nearbyTracks; nearbyTracks.reserve(500);
        int stepFar = g_Config.NMT1_StepFar, stepMid = g_Config.NMT1_StepMid, stepNear = g_Config.NMT1_StepNear;
        float distMidSq = g_Config.NMT1_DistMid * g_Config.NMT1_DistMid, distNearSq = g_Config.NMT1_DistNear * g_Config.NMT1_DistNear;

        for (size_t i = startIdx; i < endIdx; ++i) {
            auto buffer = FastParser::ReadFileToBuffer(ascFiles[i].string()); if (buffer.empty()) continue;
            char* ptr = buffer.data(); int ncols = 0, nrows = 0; double xll = 0, yll = 0, cellsize = 0, nodata = -9999;
            for(int h=0; h<6; h++) {
                ptr = FastParser::SkipWhitespace(ptr); while(*ptr && !isspace(*ptr)) ptr++;
                ptr = FastParser::SkipWhitespace(ptr); 
                if(h==0) ncols = std::atoi(ptr); else if(h==1) nrows = std::atoi(ptr); else if(h==2) xll = std::strtod(ptr, &ptr);
                else if(h==3) yll = std::strtod(ptr, &ptr); else if(h==4) cellsize = std::strtod(ptr, &ptr); else if(h==5) nodata = std::strtod(ptr, &ptr);
                if(h < 5) while(*ptr && *ptr != '\n') ptr++;
            }
            
            for (int r = 0; r < nrows; r++) {
                double currentGeoX = (yll + nrows * cellsize) - (r * cellsize);
                for (int c = 0; c < ncols; c++) {
                    bool isFar = (r % stepFar == 0 && c % stepFar == 0), isMid = (r % stepMid == 0 && c % stepMid == 0), isNear = (r % stepNear == 0 && c % stepNear == 0);
                    if (!isFar && !isMid && !isNear) { ptr = FastParser::SkipWhitespace(ptr); while (*ptr && !isspace(*ptr)) ptr++; continue; }

                    ptr = FastParser::SkipWhitespace(ptr); char* endPtr; float zVal = std::strtof(ptr, &endPtr); ptr = endPtr;
                    if (std::abs(zVal - nodata) > 0.1f) {
                        double currentGeoY = xll + (c * cellsize);
                        Vector3 pos = MathUtils::GeoToSim(currentGeoX, currentGeoY, zVal);
                        bool keep = false;
                        if (isFar) { keep = true; } else {
                            trackGrid.GetNearby(pos, nearbyTracks); float minDistSq = 1e9f;
                            for (const auto* trk : nearbyTracks) {
                                float l2 = MathUtils::DistanceSquared2D(trk->p1, trk->p2); float dSq = 0.0f;
                                if (l2 == 0.0f) { dSq = MathUtils::DistanceSquared2D(pos, trk->p1); } else {
                                    float t = ((pos.x - trk->p1.x) * (trk->p2.x - trk->p1.x) + (pos.z - trk->p1.z) * (trk->p2.z - trk->p1.z)) / l2;
                                    t = std::max(0.0f, std::min(1.0f, t));
                                    Vector3 proj = { trk->p1.x + t * (trk->p2.x - trk->p1.x), 0, trk->p1.z + t * (trk->p2.z - trk->p1.z) };
                                    dSq = MathUtils::DistanceSquared2D(pos, proj);
                                }
                                if (dSq < minDistSq) minDistSq = dSq;
                            }
                            if (isMid && minDistSq <= distMidSq) keep = true; else if (isNear && minDistSq <= distNearSq) keep = true;
                        }
                        if (keep) { threadBuffers[threadIdx].push_back({pos, true, true, false}); }
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
    while (filesProcessed < ascFiles.size()) { ShowProgress(filesProcessed, ascFiles.size(), "[WCZYTYWANIE] NMT1 LOD"); std::this_thread::sleep_for(std::chrono::milliseconds(200)); }
    if (g_Config.ProgressMode != 0) std::cout << std::endl;
    for (auto& w : workers) w.join();
    for (auto& buf : threadBuffers) outPoints.insert(outPoints.end(), buf.begin(), buf.end());
    std::cout << " -> NMT1: " << outPoints.size() << " punktow." << std::endl;
}

void LoadNMT100_TXT(const std::string& filename, std::vector<TerrainPoint>& points) {
    if (!fs::exists(filename)) return;
    std::cout << "[WCZYTYWANIE] NMT100..." << std::endl; std::ifstream file(filename); if (!file.is_open()) return;
    double gX, gY, gH; 
    while (file >> gY >> gX >> gH) { 
        Vector3 pos; if (g_Config.SwapNMT100Axes) pos = MathUtils::GeoToSim(gX, gY, (float)gH); else pos = MathUtils::GeoToSim(gY, gX, (float)gH);
        points.push_back({pos, false, true, false});
    }
}

void GenerateEmbankmentPoints(const std::vector<TrackSegment>& tracks, std::vector<TerrainPoint>& points) {
    std::cout << "[GENEROWANIE] Punkty nasypow..." << std::endl;
    PointGrid pGrid; for (const auto& p : points) pGrid.Add(p.pos);
    std::vector<TerrainPoint> newPoints;
    const float OFFSET_XZ = g_Config.EmbankmentWidth; const float MIN_DIST = g_Config.MinPointDist;
    size_t processed = 0, total = tracks.size();

    for (const auto& t : tracks) {
        processed++; if (processed % 1000 == 0) ShowProgress(processed, total, "[GENEROWANIE] Nasypy");
        
        if (t.isTunnel || t.isBridge || t.isGhost) continue;

        Vector3 dir = t.p2 - t.p1; float len = dir.Length(); if (len < 0.1f) continue;
        Vector3 dirNorm = dir.Normalized(); Vector3 perp = {-dirNorm.z, 0.0f, dirNorm.x};
        float step = g_Config.EmbankmentStep; float currentOffset = g_Config.TrackOffset;
        if (t.isPlatform) currentOffset += g_Config.PlatformExtraOffset;
        
        for (float d = 0.0f; d < len - 0.01f; d += step) {
            Vector3 centerPos = t.p1 + dirNorm * d;
            float trackHeight = MathUtils::Lerp(t.p1.y, t.p2.y, d/len);
            float terrainHeight = trackHeight - currentOffset;

            Vector3 leftPos = centerPos + perp * OFFSET_XZ; leftPos.y = terrainHeight;
            Vector3 rightPos = centerPos - perp * OFFSET_XZ; rightPos.y = terrainHeight;

            // Czwarty argument to TRUE. Ramiona nasypu są znów nietykalne (isFixed)!
            // Zapobiega to ich "topieniu się" i tworzeniu zapadlisk (bruzd) między torami.
            if (!pGrid.HasNeighbor(leftPos, MIN_DIST)) { newPoints.push_back({leftPos, true, true, true}); pGrid.Add(leftPos); }
            if (!pGrid.HasNeighbor(rightPos, MIN_DIST)) { newPoints.push_back({rightPos, true, true, true}); pGrid.Add(rightPos); }
        }
    }
    if(g_Config.ProgressMode != 0) std::cout << std::endl;
    points.insert(points.end(), newPoints.begin(), newPoints.end());
}

void ProcessTerrain(std::vector<TerrainPoint>& points, const std::vector<TrackSegment>& tracks) {
    std::cout << "[OBLICZANIE] Budowa indeksow przestrzennych..." << std::endl;
    SpatialGrid<150> fineGrid; fineGrid.Build(tracks);
    SpatialGrid<4000> wideGrid; wideGrid.Build(tracks);

    const float LIMIT_NMT1 = g_Config.LimitNMT1; const float LIMIT_NMT100_MAX = g_Config.LimitNMT100Max;
    const float SNAP_DIST = g_Config.SnapDist; const float SMOOTH_END = g_Config.SmoothEnd;

    unsigned int threads = GetThreadCount(); if (threads > 1) threads -= 1;
    std::cout << "[FILTROWANIE] Przetwarzanie terenu (" << threads << " watkow)..." << std::endl;
    std::vector<std::thread> workers; std::atomic<size_t> processed(0);
    size_t total = points.size(); size_t chunk = (total + threads - 1) / threads;

    auto workerTask = [&](size_t start, size_t end) {
        std::vector<const TrackSegment*> nearbyFine; nearbyFine.reserve(200);
        std::vector<const TrackSegment*> nearbyWide; nearbyWide.reserve(200);
        
        for (size_t i = start; i < end; ++i) {
            TerrainPoint& pt = points[i];
            
            // Zapewnia to, że szerokość podkładów i tłucznia jest perfekcyjnie zachowana.
            if (pt.isFixed) {
                pt.isValid = true; // Ramiona nasypu zawsze są poprawne
                processed++; 
                continue;
            }

            fineGrid.GetNearby(pt.pos, nearbyFine);

            float minDistBridge = 1e9f;
            const TrackSegment* nearestBridge = nullptr;
            float distToAnyTrack = 1e9f;
            float minTrackDist = 1e9f;

            for (const auto* trk : nearbyFine) {
                float dummyY;
                float d = MathUtils::GetDistanceToSegment(pt.pos, *trk, dummyY);
                if (d < distToAnyTrack) distToAnyTrack = d;

                if (trk->isBridge || trk->isTunnel) {
                    if (d < minDistBridge) { minDistBridge = d; nearestBridge = trk; }
                }
            }

            // Niszczenie śmieci laserowych na mostach (działa tylko dla punktów niezablokowanych)
            bool killPoint = false;
            if (nearestBridge != nullptr && minDistBridge <= 12.0f && pt.isNMT1) {
                float bY; MathUtils::GetDistanceToSegment(pt.pos, *nearestBridge, bY);
                if (pt.pos.y > bY - 2.5f) { killPoint = true; }
            }

            if (killPoint) { pt.isValid = false; processed++; continue; }
            
            // ZMIANA v283: Sześcienne blendowanie IDW (tylko dla naturalnych punktów terenu NMT)
            float sumWeightedHeights = 0.0f;
            float sumWeights = 0.0f;

            for (const auto* trk : nearbyFine) {
                if (trk->isBridge || trk->isTunnel || trk->isGhost) continue; 
                
                float trkY; 
                float d = MathUtils::GetDistanceToSegment(pt.pos, *trk, trkY);
                if (d < minTrackDist) minTrackDist = d;
                
                if (d <= SMOOTH_END) {
                    float currentOffset = g_Config.TrackOffset;
                    if (trk->isPlatform) currentOffset += g_Config.PlatformExtraOffset;
                    float targetH = trkY - currentOffset;

                    // Kubiczna matematyka zapewnia, że punkt NMT błyskawicznie "łapie" wysokość 
                    // najbliższego toru, a nie tego w oddali.
                    float d_clamped = std::max(0.1f, d); 
                    float weight = 1.0f / (d_clamped * d_clamped * d_clamped);
                    
                    sumWeightedHeights += targetH * weight;
                    sumWeights += weight;
                }
            }

            if (sumWeights > 0.0000001f && minTrackDist <= SMOOTH_END) {
                float blendedTargetH = sumWeightedHeights / sumWeights;
                
                float blendFactor = 1.0f; 
                if (minTrackDist > SNAP_DIST) {
                    float t = (minTrackDist - SNAP_DIST) / (SMOOTH_END - SNAP_DIST);
                    blendFactor = 1.0f - (t * t * (3.0f - 2.0f * t)); 
                }
                
                pt.pos.y = MathUtils::Lerp(pt.pos.y, blendedTargetH, blendFactor);
            }

            // Limitowanie odległościowe
            if (pt.isNMT1) { pt.isValid = (distToAnyTrack <= LIMIT_NMT1); } 
            else { 
                if (distToAnyTrack <= LIMIT_NMT1) pt.isValid = false;
                else if (distToAnyTrack <= LIMIT_NMT100_MAX) pt.isValid = true;
                else {
                    nearbyWide.clear(); wideGrid.GetNearby(pt.pos, nearbyWide);
                    bool inRange = false;
                    for (const auto* trk : nearbyWide) {
                        float dummyH;
                        if (MathUtils::GetDistanceToSegment(pt.pos, *trk, dummyH) <= LIMIT_NMT100_MAX) { inRange = true; break; }
                    }
                    pt.isValid = inRange;
                }
            }
            processed++;
        }
    };

    for (unsigned int i = 0; i < threads; ++i) {
        size_t start = i * chunk; size_t end = std::min(start + chunk, total);
        workers.emplace_back(workerTask, start, end);
    }
    while(processed < total) { ShowProgress(processed, total, "[FILTROWANIE] Postep"); std::this_thread::sleep_for(std::chrono::milliseconds(250)); }
    if (g_Config.ProgressMode != 0) std::cout << std::endl;
    for (auto& w : workers) w.join();
    points.erase(std::remove_if(points.begin(), points.end(), [](const TerrainPoint& p){ return !p.isValid; }), points.end());
}

void RemoveDuplicates(std::vector<TerrainPoint>& points) {
    std::cout << "[OBLICZANIE] Usuwanie duplikatow..." << std::endl;
    std::sort(points.begin(), points.end(), [](const TerrainPoint& a, const TerrainPoint& b) {
        if (std::abs(a.pos.x - b.pos.x) > 0.01f) return a.pos.x < b.pos.x;
        return a.pos.z < b.pos.z;
    });
    const float MERGE_DIST = g_Config.MergeClosePoints;
    points.erase(std::unique(points.begin(), points.end(), [&](const TerrainPoint& a, const TerrainPoint& b) {
        return std::abs(a.pos.x - b.pos.x) < MERGE_DIST && std::abs(a.pos.z - b.pos.z) < MERGE_DIST;
    }), points.end());
}

void ExportTerrain(std::vector<TerrainPoint>& points) {
    std::cout << "[GENEROWANIE] Triangulacja..." << std::endl;
    std::vector<double> coords; coords.reserve(points.size() * 2);
    for (const auto& p : points) { coords.push_back(p.pos.x); coords.push_back(p.pos.z); }
    delaunator::Delaunator d(coords);

    std::cout << "[GENEROWANIE] Obliczanie oswietlenia..." << std::endl;
    const float MAX_EDGE = g_Config.MaxTriangleEdge;
    std::vector<TriangleSortInfo> sortedTris; sortedTris.reserve(d.triangles.size() / 3);

    for (size_t i = 0; i < d.triangles.size(); i += 3) {
        size_t v1 = d.triangles[i], v2 = d.triangles[i+1], v3 = d.triangles[i+2];
        Vector3 p1 = points[v1].pos, p2 = points[v2].pos, p3 = points[v3].pos;
        
        if (std::sqrt(std::pow(p1.x-p2.x, 2) + std::pow(p1.z-p2.z, 2)) > MAX_EDGE || std::sqrt(std::pow(p2.x-p3.x, 2) + std::pow(p2.z-p3.z, 2)) > MAX_EDGE || std::sqrt(std::pow(p3.x-p1.x, 2) + std::pow(p3.z-p1.z, 2)) > MAX_EDGE) continue;

        Vector3 FaceNormal = (p2 - p1).Cross(p3 - p1); 
        points[v1].normal += FaceNormal; points[v2].normal += FaceNormal; points[v3].normal += FaceNormal;
        TriangleSortInfo t; t.v1 = v1; t.v2 = v2; t.v3 = v3; t.cx = (p1.x + p2.x + p3.x) / 3.0f; t.cz = (p1.z + p2.z + p3.z) / 3.0f; sortedTris.push_back(t);
    }
    for(auto& p : points) p.normal = p.normal.Normalized();

    std::cout << "[GENEROWANIE] Sortowanie trojkatow..." << std::endl;
    std::sort(sortedTris.begin(), sortedTris.end(), [](const TriangleSortInfo& a, const TriangleSortInfo& b) {
        int za = (int)(a.cz / 200.0f), zb = (int)(b.cz / 200.0f); if (za != zb) return za < zb; return a.cx < b.cx;
    });

    auto now_c = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

    if (g_Config.ExportSCM) {
        std::cout << "[EKSPORT SCM] Plik: " << g_Config.OutputSCM << "..." << std::endl;
        std::ofstream out(g_Config.OutputSCM); out.imbue(std::locale("C")); out << std::fixed << std::setprecision(2);
        out << "// Generated by TerenAI " << PROG_VERSION << "\n// Date: " << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S") << "\n// Offset: East=" << g_Config.OffsetEast << ", North=" << g_Config.OffsetNorth << "\n\n";

        size_t nodeCounter = 0, written = 0, totalTris = sortedTris.size();
        for (size_t i = 0; i < sortedTris.size(); i += 4) {
            written += 4; if (written % 10000 == 0) ShowProgress(written, totalTris, "[EKSPORT SCM] Zapis");
            out << "node -1 0 teren_" << nodeCounter++ << " triangles grass\n";
            size_t batchEnd = std::min(i + 4, sortedTris.size());
            for (size_t k = i; k < batchEnd; k++) {
                const auto& t = sortedTris[k]; size_t idx[] = {t.v1, t.v2, t.v3};
                for (int v = 0; v < 3; v++) {
                    const auto& p = points[idx[v]];
                    out << p.pos.x << " " << p.pos.y << " " << p.pos.z << " " << p.normal.x << " " << p.normal.y << " " << p.normal.z << " " << p.pos.x * 0.04f << " " << p.pos.z * 0.04f;
                    if (k == batchEnd - 1 && v == 2) out << "\n"; else out << " end\n";
                }
            }
            out << "endtri\n\n";
        }
        if (g_Config.ProgressMode != 0) std::cout << std::endl; out.close(); std::cout << " -> Zapisano " << nodeCounter << " wezlow w SCM." << std::endl;
    }

    if (g_Config.ExportE3D) {
        std::cout << "[EKSPORT E3D] Plik: " << g_Config.OutputE3D << "..." << std::endl; E3DChunkWriter tex0, nam0, vnt0, idx4, sub0, e3d0;
        tex0.writeID("TEX0"); tex0.writeU32(0); tex0.data.push_back(0); std::string tName = "grass"; tex0.data.insert(tex0.data.end(), tName.begin(), tName.end()); tex0.data.push_back(0); tex0.pad(); tex0.finalizeLength();
        nam0.writeID("NAM0"); nam0.writeU32(0); std::string sName = "teren"; nam0.data.insert(nam0.data.end(), sName.begin(), sName.end()); nam0.data.push_back(0); nam0.pad(); nam0.finalizeLength();
        vnt0.writeID("VNT0"); vnt0.writeU32(0);
        for (const auto& p : points) { vnt0.writeF32(p.pos.x); vnt0.writeF32(p.pos.y); vnt0.writeF32(p.pos.z); vnt0.writeF32(p.normal.x); vnt0.writeF32(p.normal.y); vnt0.writeF32(p.normal.z); vnt0.writeF32(p.pos.x * 0.04f); vnt0.writeF32(p.pos.z * 0.04f); }
        vnt0.pad(); vnt0.finalizeLength();
        idx4.writeID("IDX4"); idx4.writeU32(0);
        for (const auto& t : sortedTris) { idx4.writeU32(static_cast<uint32_t>(t.v1)); idx4.writeU32(static_cast<uint32_t>(t.v2)); idx4.writeU32(static_cast<uint32_t>(t.v3)); }
        idx4.pad(); idx4.finalizeLength();
        sub0.writeID("SUB0"); sub0.writeU32(0); sub0.writeI32(-1); sub0.writeI32(-1); sub0.writeI32(4); sub0.writeI32(0); sub0.writeI32(0); sub0.writeI32(16); sub0.writeI32(-1); sub0.writeI32(static_cast<int32_t>(points.size())); sub0.writeI32(0); sub0.writeI32(1); sub0.writeF32(0.0f); sub0.writeF32(0.0f); sub0.writeF32(0); sub0.writeF32(0); sub0.writeF32(0); sub0.writeF32(0); sub0.writeF32(1.0f); sub0.writeF32(1.0f); sub0.writeF32(1.0f); sub0.writeF32(1.0f); sub0.writeF32(0); sub0.writeF32(0); sub0.writeF32(0); sub0.writeF32(0); sub0.writeF32(0); sub0.writeF32(0); sub0.writeF32(0); sub0.writeF32(0); sub0.writeF32(1.0f); sub0.writeF32(100000000.0f); sub0.writeF32(0.0f); sub0.writeZeroes(32); sub0.writeI32(static_cast<int32_t>(sortedTris.size() * 3)); sub0.writeI32(0); sub0.writeF32(1.0f); sub0.writeZeroes(88); sub0.finalizeLength();
        e3d0.writeID("E3D0"); e3d0.writeU32(8 + static_cast<uint32_t>(sub0.data.size() + vnt0.data.size() + idx4.data.size() + tex0.data.size() + nam0.data.size())); e3d0.data.insert(e3d0.data.end(), sub0.data.begin(), sub0.data.end()); e3d0.data.insert(e3d0.data.end(), vnt0.data.begin(), vnt0.data.end()); e3d0.data.insert(e3d0.data.end(), idx4.data.begin(), idx4.data.end()); e3d0.data.insert(e3d0.data.end(), tex0.data.begin(), tex0.data.end()); e3d0.data.insert(e3d0.data.end(), nam0.data.begin(), nam0.data.end());
        std::ofstream file(g_Config.OutputE3D, std::ios::binary); file.write(reinterpret_cast<const char*>(e3d0.data.data()), e3d0.data.size()); file.close(); std::cout << " -> Zapisano binarny plik E3D. Rozmiar: " << std::fixed << std::setprecision(1) << (e3d0.data.size() / 1024.0 / 1024.0) << " MB." << std::endl;
    }
}

void ParseArgs(int argc, char* argv[]) { for (int i = 1; i < argc; ++i) { std::string arg = argv[i]; if (arg == "-help" || arg == "/?") { PrintConfig(); exit(0); } } }

int main(int argc, char* argv[]) {
    std::locale::global(std::locale("C")); std::cout << "--- terenAI " << PROG_VERSION << " --- \n"; LoadIniConfig("terenAI.ini"); ParseArgs(argc, argv); 
    
    if (g_Config.ScnFiles.empty()) { std::cerr << "BLAD: Nie zdefiniowano zadnych plikow w FileSCN!\n"; return 1; }
    ParseSCNOffsets(g_Config.ScnFiles); PrintConfig();
    
    if (!g_Config.ExportSCM && !g_Config.ExportE3D) { std::cerr << "BLAD: Wszystkie opcje eksportu wylaczone.\n"; return 1; }
    if (!g_Config.OffsetsLoaded) { std::cout << "Podaj Offset East (km): "; std::cin >> g_Config.OffsetEast; std::cout << "Podaj Offset North (km): "; std::cin >> g_Config.OffsetNorth; g_Config.OffsetEast *= 1000.0; g_Config.OffsetNorth *= 1000.0; }
    
    std::vector<TrackSegment> tracks; 
    for (const auto& filename : g_Config.ScnFiles) { LoadTracksFromSCN(filename, tracks); }
    if (tracks.empty()) { std::cerr << "BLAD: Brak torow!" << std::endl; return 1; }
    
    FlagGhostTracks(tracks);

    std::vector<TerrainPoint> points;
    LoadNMT1_Turbo(g_Config.DirNMT1, points, tracks); 
    LoadNMT100_TXT(g_Config.FileNMT100, points); 
    GenerateEmbankmentPoints(tracks, points); 
    RemoveDuplicates(points); 
    ProcessTerrain(points, tracks); 
    ExportTerrain(points);
    
    std::cout << "Gotowe.\nNacisnij Enter, aby zamknac okno..." << std::endl; std::cin.get(); return 0;
}