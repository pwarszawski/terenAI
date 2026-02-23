// Ten plik zawiera kod na licencji MIT - delaunator.hpp
// 
// Copyright (c) 2018 Volodymyr Bilonenko
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

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

// -----------------------------------------------------------
// Plik delaunator.hpp musi być w katalogu projektu
// -----------------------------------------------------------
#include "delaunator.hpp" 

namespace fs = std::filesystem;

// -----------------------------------------------------------
// Konfiguracja i struktury danych
// -----------------------------------------------------------

const std::string PROG_VERSION = "v264 (Binary E3D Engine)";

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
    
    // Parametry nasypów
    float EmbankmentWidth = 2.60f; 
    float EmbankmentStep = 5.0f;   
    float MinPointDist = 1.0f;     
    float MergeClosePoints = 0.15f; 

    // Opcje eksportu
    bool ExportSCN = true;
    bool ExportE3D = false;

    // Nazwy plików
    std::string DirNMT1 = "NMT1";
    std::string FileNMT100 = "NMT100.txt";
    std::string FileSCN = "EXPORT.SCN";
    std::string FileOutputSCN = "teren.scm";
    std::string FileOutputE3D = "teren.e3d";

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
        x += other.x; y += other.y; z += other.z; return *this;
    }
    float Length() const { return std::sqrt(x*x + y*y + z*z); }
    Vector3 Normalized() const {
        float len = Length(); if (len < 0.00001f) return {0,1,0}; return {x/len, y/len, z/len};
    }
    Vector3 Cross(const Vector3& other) const {
        return { y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x };
    }
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
    Vector3 p1, p2; bool isTunnel; bool isBridge; float minX, maxX, minZ, maxZ; 
    void CalcBounds() {
        minX = std::min(p1.x, p2.x); maxX = std::max(p1.x, p2.x);
        minZ = std::min(p1.z, p2.z); maxZ = std::max(p1.z, p2.z);
    }
};

struct TriangleSortInfo {
    size_t v1, v2, v3;
    float cx, cz;
};

// -----------------------------------------------------------
// STRUKTURY BINARNE E3D (Ścisłe wyrównanie pamięci!)
// -----------------------------------------------------------
#pragma pack(push, 1)
struct Sub0Entry {
    int next;               // 0: Numer nastepnego submodelu (-1 brak)
    int child;              // 4: Numer potomnego (-1 brak)
    int type;               // 8: GL_TRIANGLES = 4
    int name_id;            // 12: Numer nazwy w NAM0
    int anim_type;          // 16: 0 = brak
    int flags;              // 20: Flagi (bit 4 = nieprzezroczysty)
    int matrix_id;          // 24: -1 = jednostkowa
    int vertex_count;       // 28: Liczba wierzcholków (w VNT0)
    int first_vertex;       // 32: Pierwszy wierzcholek w buforze
    int material_id;        // 36: 1 = pierwszy material z TEX0
    float min_darkness;     // 40: 0.0f
    float light_threshold;  // 44: 0.0f
    float ambient[4];       // 48
    float diffuse[4];       // 64: {1,1,1,1}
    float specular[4];      // 80
    float emission[4];      // 96
    float line_size;        // 112: 0.0f
    float max_vis_dist_sq;  // 116: Zasięg rysowania
    float min_vis_dist_sq;  // 120: 0.0f
    float light_params[8];  // 124
    int index_count;        // 156: 0 (brak indeksów, czysty bufor)
    int first_index;        // 160: 0
    float diffuse_mult;     // 164: 1.0f
    char reserved[88];      // 168: Wypelnione zerami
};
#pragma pack(pop)

// -----------------------------------------------------------
// Narzędzia pomocnicze
// -----------------------------------------------------------

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
    } else if (g_Config.ProgressMode == 2) {
        int barWidth = 30; std::cout << "\r" << label << ": [";
        int pos = barWidth * percent / 100.0f;
        for (int i = 0; i < barWidth; ++i) {
            if (i < pos) std::cout << "="; else if (i == pos) std::cout << ">"; else std::cout << " ";
        }
        std::cout << "] " << int(percent) << "% " << std::flush;
    }
}

unsigned int GetThreadCount() {
    unsigned int hw = std::thread::hardware_concurrency(); if (hw == 0) hw = 4;
    int percent = g_Config.CpuUsagePercent; if (percent < 1) percent = 1; if (percent > 100) percent = 100;
    unsigned int target = static_cast<unsigned int>(hw * (percent / 100.0f));
    if (target < 1) target = 1; return target;
}

class FastParser {
public:
    static std::vector<char> ReadFileToBuffer(const std::string& path) {
        std::ifstream file(path, std::ios::binary | std::ios::ate);
        if (!file) return {};
        std::streamsize size = file.tellg(); file.seekg(0, std::ios::beg);
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
        float u = 1.0f - t; float tt = t * t, uu = u * u, uuu = uu * u, ttt = tt * t;
        Vector3 p = p1 * uuu; p = p + c1 * (3 * uu * t); p = p + c2 * (3 * u * tt); p = p + p2 * ttt; return p;
    }
    static float DistanceSquared2D(const Vector3& a, const Vector3& b) {
        float dx = a.x - b.x; float dz = a.z - b.z; return dx * dx + dz * dz;
    }
    static float GetDistanceToSegment(const Vector3& p, const TrackSegment& seg, float& outTrackY) {
        float l2 = DistanceSquared2D(seg.p1, seg.p2);
        if (l2 == 0.0f) { outTrackY = seg.p1.y; return std::sqrt(DistanceSquared2D(p, seg.p1)); }
        float t = ((p.x - seg.p1.x) * (seg.p2.x - seg.p1.x) + (p.z - seg.p1.z) * (seg.p2.z - seg.p1.z)) / l2;
        t = std::max(0.0f, std::min(1.0f, t));
        Vector3 proj; proj.x = seg.p1.x + t * (seg.p2.x - seg.p1.x); proj.z = seg.p1.z + t * (seg.p2.z - seg.p1.z);
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

// -----------------------------------------------------------
// Siatki przestrzenne
// -----------------------------------------------------------

template <int CellSize>
class SpatialGrid {
    struct CellKey { int x, z; bool operator==(const CellKey& o) const { return x == o.x && z == o.z; } };
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
                for (int z = minIz; z <= maxIz; ++z) grid[{x, z}].push_back(&t);
            }
        }
    }
    void GetNearby(const Vector3& p, std::vector<const TrackSegment*>& out) const {
        out.clear(); int cx = GetIndex(p.x); int cz = GetIndex(p.z);
        int range = (CellSize > 1000) ? 4 : 3;
        for (int x = -range; x <= range; x++) {
            for (int z = -range; z <= range; z++) {
                CellKey k = { cx + x, cz + z }; auto it = grid.find(k);
                if (it != grid.end()) out.insert(out.end(), it->second.begin(), it->second.end());
            }
        }
        if (!out.empty()) { std::sort(out.begin(), out.end()); auto last = std::unique(out.begin(), out.end()); out.erase(last, out.end()); }
    }
};

class PointGrid {
    struct CellKey { int x, z; bool operator==(const CellKey& o) const { return x == o.x && z == o.z; } };
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
                    for (const auto& other : it->second) { if (MathUtils::DistanceSquared2D(p, other) < r2) return true; }
                }
            }
        }
        return false;
    }
};

// -----------------------------------------------------------
// Odczyt pliku .ini oraz offsetu z pliku SCN
// -----------------------------------------------------------

void LoadIniConfig(const std::string& filename) {
    if (!fs::exists(filename)) { std::cerr << "BLAD: Brak pliku konfiguracyjnego: " << filename << std::endl; exit(1); }
    std::ifstream file(filename); std::string line;
    while (std::getline(file, line)) {
        line = Trim(line);
        if (line.empty() || line[0] == ';' || line[0] == '#' || line[0] == '[') continue;
        size_t eqPos = line.find('='); if (eqPos == std::string::npos) continue;
        std::string key = Trim(line.substr(0, eqPos)); std::string val = Trim(line.substr(eqPos + 1));
        try {
            if (key == "FileSCN") g_Config.FileSCN = val;
            else if (key == "DirNMT1") g_Config.DirNMT1 = val;
            else if (key == "FileNMT100") g_Config.FileNMT100 = val;
            
            else if (key == "ExportSCN") g_Config.ExportSCN = (std::stoi(val) != 0);
            else if (key == "ExportE3D") g_Config.ExportE3D = (std::stoi(val) != 0);
            else if (key == "OutputSCN") g_Config.FileOutputSCN = val;
            else if (key == "OutputE3D") g_Config.FileOutputE3D = val;

            else if (key == "LimitNMT1") g_Config.LimitNMT1 = std::stof(val);
            else if (key == "LimitNMT100") g_Config.LimitNMT100Max = std::stof(val);
            else if (key == "SnapDist") g_Config.SnapDist = std::stof(val);
            else if (key == "SmoothDist") g_Config.SmoothEnd = std::stof(val);
            else if (key == "MaxTriangleEdge") g_Config.MaxTriangleEdge = std::stof(val);
            else if (key == "TrackOffset") g_Config.TrackOffset = std::stof(val);
            else if (key == "EmbankmentWidth") g_Config.EmbankmentWidth = std::stof(val);
            else if (key == "EmbankmentStep") g_Config.EmbankmentStep = std::stof(val);
            else if (key == "MinPointDist") g_Config.MinPointDist = std::stof(val);
            else if (key == "MergeClosePoints") g_Config.MergeClosePoints = std::stof(val);
            else if (key == "CpuUsagePercent") g_Config.CpuUsagePercent = std::stoi(val);
            else if (key == "ProgressMode") g_Config.ProgressMode = std::stoi(val);
            else if (key == "SwapNMT100Axes") g_Config.SwapNMT100Axes = (std::stoi(val) != 0);
        } catch (...) {}
    }
}

bool ParseSCNOffsets(const std::string& filename) {
    std::ifstream file(filename); if (!file.is_open()) return false;
    std::string line;
    for (int i = 0; i < 50; i++) {
        if (!std::getline(file, line)) break;
        if (line.find("//$g") != std::string::npos) {
            std::stringstream ss(line); std::string tag, sys; double kmEast, kmNorth; ss >> tag; 
            if (tag == "//$g") { ss >> sys; if (ss >> kmEast >> kmNorth) { g_Config.OffsetEast = kmEast * 1000.0; g_Config.OffsetNorth = kmNorth * 1000.0; g_Config.OffsetsLoaded = true; return true; } }
        }
    }
    return false;
}

// -----------------------------------------------------------
// Odczyt danych z plików SCN, NMT1 i NMT100
// -----------------------------------------------------------

void LoadTracksFromSCN(const std::string& filename, std::vector<TrackSegment>& tracks) {
    std::ifstream file(filename); if (!file.is_open()) { std::cerr << "BLAD: Nie mozna otworzyc pliku SCN!" << std::endl; return; }
    std::string line; Vector3 p1, cv1, cv2, p2; bool foundP1=false, foundCV1=false, foundCV2=false, foundP2=false;
    bool inTrack = false; bool isTunnelTrack = false; bool isBridgeTrack = false; int totalSegments = 0;
    std::cout << "[WCZYTYWANIE] Tory (Normal + Switch)..." << std::endl;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        if (line.find("node") != std::string::npos && line.find("track") != std::string::npos) {
            if (line.find("normal") != std::string::npos || line.find("switch") != std::string::npos) {
                inTrack = true; foundP1 = foundCV1 = foundCV2 = foundP2 = false; 
                isTunnelTrack = (line.find("tunnel") != std::string::npos);
                isBridgeTrack = (line.find("bridge") != std::string::npos);
            } continue;
        }
        if (inTrack) {
            if (line.find("endtrack") != std::string::npos) { inTrack = false; continue; }
            char firstChar = ' '; for(char c : line) { if(!isspace(c)) { firstChar = c; break; } }
            if (!isdigit(firstChar) && firstChar != '-') continue;
            std::stringstream ss(line); float x, y, z; 
            if (!foundP1) { if (ss >> x >> y >> z) { p1 = {x, y, z}; foundP1 = true; } }
            else if (!foundCV1) { if (ss >> x >> y >> z) { cv1 = {x, y, z}; foundCV1 = true; } }
            else if (!foundCV2) { if (ss >> x >> y >> z) { cv2 = {x, y, z}; foundCV2 = true; } }
            else if (!foundP2) {
                if (ss >> x >> y >> z) { 
                    p2 = {x, y, z}; foundP2 = true;
                    float approxLen = std::sqrt(std::pow(p1.x-p2.x, 2) + std::pow(p1.y-p2.y, 2) + std::pow(p1.z-p2.z, 2));
                    int segments = static_cast<int>(approxLen / 5.0f);
                    if (segments < 2) segments = 2; if (segments > 48) segments = 48;
                    for (int i = 1; i <= segments; i++) {
                        float t = (float)i / (float)segments;
                        Vector3 curr = MathUtils::CubicBezier(t, p1, cv1, cv2, p2);
                        Vector3 prev = (i == 1) ? p1 : MathUtils::CubicBezier((float)(i - 1) / segments, p1, cv1, cv2, p2);
                        TrackSegment tSeg; tSeg.p1 = prev; tSeg.p2 = curr; 
                        tSeg.isTunnel = isTunnelTrack; tSeg.isBridge = isBridgeTrack; tSeg.CalcBounds();
                        tracks.push_back(tSeg); totalSegments++;
                    }
                }
            }
        }
    }
    std::cout << " -> Zaladowano " << totalSegments << " segmentow." << std::endl;
}

void LoadNMT1_Turbo(const std::string& folderPath, std::vector<TerrainPoint>& outPoints) {
    if (!fs::exists(folderPath)) return;
    std::vector<fs::path> ascFiles;
    for (const auto& entry : fs::directory_iterator(folderPath)) { if (entry.path().extension() == ".asc") ascFiles.push_back(entry.path()); }
    if (ascFiles.empty()) return;

    unsigned int threadsCount = GetThreadCount(); if (threadsCount > 2) threadsCount -= 2; else threadsCount = 1;
    std::cout << "[WCZYTYWANIE] NMT1 (" << ascFiles.size() << " plikow)..." << std::endl;
    std::atomic<size_t> filesProcessed(0); std::vector<std::vector<TerrainPoint>> threadBuffers(threadsCount);

    auto worker = [&](int threadIdx, size_t startIdx, size_t endIdx) {
        threadBuffers[threadIdx].reserve(500000); 
        for (size_t i = startIdx; i < endIdx; ++i) {
            auto buffer = FastParser::ReadFileToBuffer(ascFiles[i].string()); if (buffer.empty()) continue;
            char* ptr = buffer.data(); int ncols=0, nrows=0; double xll=0, yll=0, cellsize=0, nodata=-9999;
            for(int h=0; h<6; h++) {
                ptr = FastParser::SkipWhitespace(ptr); while(*ptr && !isspace(*ptr)) ptr++;
                ptr = FastParser::SkipWhitespace(ptr); 
                if(h==0) ncols=std::atoi(ptr); else if(h==1) nrows=std::atoi(ptr); else if(h==2) xll=std::strtod(ptr, &ptr);
                else if(h==3) yll=std::strtod(ptr, &ptr); else if(h==4) cellsize=std::strtod(ptr, &ptr); else if(h==5) nodata=std::strtod(ptr, &ptr);
                if(h < 5) while(*ptr && *ptr != '\n') ptr++;
            }
            for (int r = 0; r < nrows; r++) {
                double currentGeoX = (yll + nrows * cellsize) - (r * cellsize);
                if (r % 10 != 0) { for (int c=0; c<ncols; c++) { ptr = FastParser::SkipWhitespace(ptr); while (*ptr && !isspace(*ptr)) ptr++; } } 
                else {
                    for (int c=0; c<ncols; c++) {
                        ptr = FastParser::SkipWhitespace(ptr);
                        if (c % 10 == 0) {
                            char* endPtr; float zVal = std::strtof(ptr, &endPtr); ptr = endPtr;
                            if (std::abs(zVal - nodata) > 0.1f) {
                                double currentGeoY = xll + (c * cellsize);
                                threadBuffers[threadIdx].push_back({MathUtils::GeoToSim(currentGeoX, currentGeoY, zVal), true, true, false});
                            }
                        } else { while (*ptr && !isspace(*ptr)) ptr++; }
                    }
                }
            } filesProcessed++;
        }
    };

    std::vector<std::thread> workers; size_t batchSize = (ascFiles.size() + threadsCount - 1) / threadsCount;
    for (unsigned int t = 0; t < threadsCount; ++t) {
        size_t start = t * batchSize; size_t end = std::min(start + batchSize, ascFiles.size());
        if (start < end) workers.emplace_back(worker, t, start, end);
    }
    while (filesProcessed < ascFiles.size()) { ShowProgress(filesProcessed, ascFiles.size(), "[WCZYTYWANIE] NMT1"); std::this_thread::sleep_for(std::chrono::milliseconds(200)); }
    if (g_Config.ProgressMode != 0) std::cout << std::endl;
    for (auto& w : workers) w.join();
    for (auto& buf : threadBuffers) outPoints.insert(outPoints.end(), buf.begin(), buf.end());
    std::cout << " -> NMT1: " << outPoints.size() << " punktow." << std::endl;
}

void LoadNMT100_TXT(const std::string& filename, std::vector<TerrainPoint>& points) {
    if (!fs::exists(filename)) return;
    std::cout << "[WCZYTYWANIE] NMT100..." << std::endl;
    std::ifstream file(filename); if (!file.is_open()) return;
    double gX, gY, gH; 
    while (file >> gY >> gX >> gH) { 
        Vector3 pos; if (g_Config.SwapNMT100Axes) pos = MathUtils::GeoToSim(gX, gY, (float)gH); else pos = MathUtils::GeoToSim(gY, gX, (float)gH);
        points.push_back({pos, false, true, false});
    }
}

// -----------------------------------------------------------
// Generowanie punktów nasypów wzdłuż torów
// -----------------------------------------------------------

void GenerateEmbankmentPoints(const std::vector<TrackSegment>& tracks, std::vector<TerrainPoint>& points) {
    std::cout << "[GENEROWANIE] Punkty nasypow..." << std::endl;
    PointGrid pGrid; for (const auto& p : points) pGrid.Add(p.pos);
    std::vector<TerrainPoint> newPoints;
    const float OFFSET_Y = g_Config.TrackOffset;
    const float OFFSET_XZ = g_Config.EmbankmentWidth;
    const float MIN_DIST = g_Config.MinPointDist;
    size_t processed = 0; size_t total = tracks.size();

    for (const auto& t : tracks) {
        processed++; if (processed % 1000 == 0) ShowProgress(processed, total, "[GENEROWANIE] Nasypy");
        if (t.isTunnel || t.isBridge) continue;
        Vector3 dir = t.p2 - t.p1; float len = dir.Length(); if (len < 0.1f) continue;
        Vector3 dirNorm = dir.Normalized(); Vector3 perp = {-dirNorm.z, 0.0f, dirNorm.x};
        float step = g_Config.EmbankmentStep;
        for (float d = 0.0f; d < len - 0.01f; d += step) {
            Vector3 centerPos = t.p1 + dirNorm * d;
            float trackHeight = MathUtils::Lerp(t.p1.y, t.p2.y, d/len);
            float terrainHeight = trackHeight - OFFSET_Y; 
            Vector3 leftPos = centerPos + perp * OFFSET_XZ; leftPos.y = terrainHeight;
            Vector3 rightPos = centerPos - perp * OFFSET_XZ; rightPos.y = terrainHeight;
            if (!pGrid.HasNeighbor(leftPos, MIN_DIST)) { newPoints.push_back({leftPos, true, true, true}); pGrid.Add(leftPos); }
            if (!pGrid.HasNeighbor(rightPos, MIN_DIST)) { newPoints.push_back({rightPos, true, true, true}); pGrid.Add(rightPos); }
        }
    }
    if(g_Config.ProgressMode != 0) std::cout << std::endl;
    std::cout << " -> Dodano " << newPoints.size() << " punktow kotwiczacych." << std::endl;
    points.insert(points.end(), newPoints.begin(), newPoints.end());
}

// -----------------------------------------------------------
// Przetwarzanie terenu - filtrowanie punktów, wygładzanie
// ----------------------------------------------------------

void ProcessTerrain(std::vector<TerrainPoint>& points, const std::vector<TrackSegment>& tracks) {
    std::cout << "[OBLICZANIE] Budowa indeksow przestrzennych..." << std::endl;
    SpatialGrid<150> fineGrid; fineGrid.Build(tracks);
    SpatialGrid<4000> wideGrid; wideGrid.Build(tracks);

    const float LIMIT_NMT1 = g_Config.LimitNMT1; const float LIMIT_NMT100_MAX = g_Config.LimitNMT100Max;
    const float SNAP_DIST = g_Config.SnapDist; const float SMOOTH_END = g_Config.SmoothEnd;
    const float TRACK_OFFSET = g_Config.TrackOffset;

    unsigned int threads = GetThreadCount(); if (threads > 1) threads -= 1;
    std::cout << "[FILTROWANIE] Przetwarzanie terenu (" << threads << " watkow)..." << std::endl;
    std::vector<std::thread> workers; std::atomic<size_t> processed(0);
    size_t total = points.size(); size_t chunk = (total + threads - 1) / threads;

    auto workerTask = [&](size_t start, size_t end) {
        std::vector<const TrackSegment*> nearbyFine; nearbyFine.reserve(200);
        std::vector<const TrackSegment*> nearbyWide; nearbyWide.reserve(200);
        for (size_t i = start; i < end; ++i) {
            TerrainPoint& pt = points[i];
            if (pt.isFixed) { pt.isValid = true; processed++; continue; }
            fineGrid.GetNearby(pt.pos, nearbyFine);
            float absMinDist = 1e9f; float sumWeightedHeights = 0.0f; float sumWeights = 0.0f;
            for (const auto* trk : nearbyFine) {
                float trkY; float d = MathUtils::GetDistanceToSegment(pt.pos, *trk, trkY);
                if (d < absMinDist) absMinDist = d;
                if (trk->isTunnel || trk->isBridge) continue;
                if (d <= SMOOTH_END) {
                    float weight = (1.0f - (d / SMOOTH_END)); weight = weight * weight; 
                    float targetForThisTrack = trkY - TRACK_OFFSET; 
                    sumWeightedHeights += targetForThisTrack * weight; sumWeights += weight;
                }
            }
            if (absMinDist <= SMOOTH_END && sumWeights > 0.0001f) {
                float weightedTargetH = sumWeightedHeights / sumWeights;
                if (absMinDist <= SNAP_DIST) { pt.pos.y = weightedTargetH; } 
                else { float blendFactor = (absMinDist - SNAP_DIST) / (SMOOTH_END - SNAP_DIST); pt.pos.y = MathUtils::Lerp(weightedTargetH, pt.pos.y, blendFactor); }
            }
            if (pt.isNMT1) {
                if (absMinDist <= LIMIT_NMT1) pt.isValid = true; else pt.isValid = false;
            } else { 
                if (absMinDist <= LIMIT_NMT1) pt.isValid = false;
                else {
                    if (absMinDist <= LIMIT_NMT100_MAX) pt.isValid = true;
                    else {
                        nearbyWide.clear(); wideGrid.GetNearby(pt.pos, nearbyWide); bool inRange = false;
                        for (const auto* trk : nearbyWide) {
                            float dummyH; if (MathUtils::GetDistanceToSegment(pt.pos, *trk, dummyH) <= LIMIT_NMT100_MAX) { inRange = true; break; }
                        }
                        pt.isValid = inRange;
                    }
                }
            }
            processed++;
        }
    };
    for (unsigned int i = 0; i < threads; ++i) {
        size_t start = i * chunk; size_t end = std::min(start + chunk, total); workers.emplace_back(workerTask, start, end);
    }
    while(processed < total) { ShowProgress(processed, total, "[FILTROWANIE] Postep"); std::this_thread::sleep_for(std::chrono::milliseconds(250)); }
    if (g_Config.ProgressMode != 0) std::cout << std::endl;
    for (auto& w : workers) w.join();
    auto it = std::remove_if(points.begin(), points.end(), [](const TerrainPoint& p){ return !p.isValid; });
    points.erase(it, points.end());
}

void RemoveDuplicates(std::vector<TerrainPoint>& points) {
    std::cout << "[OBLICZANIE] Usuwanie duplikatow..." << std::endl;
    std::sort(points.begin(), points.end(), [](const TerrainPoint& a, const TerrainPoint& b) {
        if (std::abs(a.pos.x - b.pos.x) > 0.01f) return a.pos.x < b.pos.x; return a.pos.z < b.pos.z;
    });
    const float MERGE_DIST = g_Config.MergeClosePoints;
    auto last = std::unique(points.begin(), points.end(), [&](const TerrainPoint& a, const TerrainPoint& b) {
        return std::abs(a.pos.x - b.pos.x) < MERGE_DIST && std::abs(a.pos.z - b.pos.z) < MERGE_DIST;
    });
    points.erase(last, points.end());
}

// -----------------------------------------------------------
// Wspólna funkcja Triangulacji i Obliczania Cieniowania
// -----------------------------------------------------------

void ProcessGeometry(std::vector<TerrainPoint>& points, std::vector<TriangleSortInfo>& outTris) {
    std::cout << "[GENEROWANIE] Triangulacja..." << std::endl;
    std::vector<double> coords; coords.reserve(points.size() * 2);
    for (const auto& p : points) { coords.push_back(p.pos.x); coords.push_back(p.pos.z); }
    delaunator::Delaunator d(coords);

    std::cout << "[GENEROWANIE] Obliczanie oswietlenia (Smooth Shading)..." << std::endl;
    for(auto& p : points) p.normal = {0,0,0};

    const float MAX_EDGE = g_Config.MaxTriangleEdge;
    outTris.reserve(d.triangles.size() / 3);

    for (size_t i = 0; i < d.triangles.size(); i += 3) {
        size_t v1 = d.triangles[i]; size_t v2 = d.triangles[i+1]; size_t v3 = d.triangles[i+2];
        Vector3 p1 = points[v1].pos; Vector3 p2 = points[v2].pos; Vector3 p3 = points[v3].pos;
        
        float d1 = std::sqrt(std::pow(p1.x-p2.x, 2) + std::pow(p1.z-p2.z, 2));
        float d2 = std::sqrt(std::pow(p2.x-p3.x, 2) + std::pow(p2.z-p3.z, 2));
        float d3 = std::sqrt(std::pow(p3.x-p1.x, 2) + std::pow(p3.z-p1.z, 2));
        if (d1 > MAX_EDGE || d2 > MAX_EDGE || d3 > MAX_EDGE) continue;

        Vector3 U = p2 - p1; Vector3 V = p3 - p1;
        Vector3 FaceNormal = U.Cross(V); 
        points[v1].normal += FaceNormal; points[v2].normal += FaceNormal; points[v3].normal += FaceNormal;

        TriangleSortInfo t; t.v1 = v1; t.v2 = v2; t.v3 = v3;
        t.cx = (p1.x + p2.x + p3.x) / 3.0f; t.cz = (p1.z + p2.z + p3.z) / 3.0f;
        outTris.push_back(t);
    }
    for(auto& p : points) p.normal = p.normal.Normalized();
}

// -----------------------------------------------------------
// ZAPIS SCM (Tekstowy)
// -----------------------------------------------------------

void SaveSCM(const std::string& filename, const std::vector<TerrainPoint>& points, std::vector<TriangleSortInfo> sortedTris) {
    std::cout << "[ZAPISYWANIE] Plik tekstowy SCM: " << filename << "..." << std::endl;
    
    // Sortowanie SCM wzdluz osi Z (zgodnie z oryginalem)
    std::sort(sortedTris.begin(), sortedTris.end(), [](const TriangleSortInfo& a, const TriangleSortInfo& b) {
        int za = (int)(a.cz / 200.0f); int zb = (int)(b.cz / 200.0f);
        if (za != zb) return za < zb; return a.cx < b.cx;
    });

    std::ofstream out(filename);
    out.imbue(std::locale("C")); out << std::fixed << std::setprecision(2);

    auto now = std::chrono::system_clock::now(); std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    out << "// Generated by TerenEU Generator " << PROG_VERSION << "\n";
    out << "// Date: " << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S") << "\n";
    out << "// Offset: East=" << g_Config.OffsetEast << ", North=" << g_Config.OffsetNorth << "\n\n";

    const size_t TRIANGLES_PER_NODE = 4; size_t nodeCounter = 0; size_t written = 0; size_t totalTris = sortedTris.size();

    for (size_t i = 0; i < sortedTris.size(); i += TRIANGLES_PER_NODE) {
        written += TRIANGLES_PER_NODE;
        if (written % 10000 == 0) ShowProgress(written, totalTris, "[ZAPISYWANIE SCM] Postep");

        out << "node -1 0 teren_" << nodeCounter++ << " triangles grass\n";
        size_t batchEnd = std::min(i + TRIANGLES_PER_NODE, sortedTris.size());
        
        for (size_t k = i; k < batchEnd; k++) {
            const auto& t = sortedTris[k]; size_t idx[] = {t.v1, t.v2, t.v3};
            for (int v = 0; v < 3; v++) {
                const auto& p = points[idx[v]]; float u_tex = p.pos.x * 0.04f; float v_tex = p.pos.z * 0.04f;
                out << p.pos.x << " " << p.pos.y << " " << p.pos.z << " " << p.normal.x << " " << p.normal.y << " " << p.normal.z << " " << u_tex << " " << v_tex;
                if (k == batchEnd - 1 && v == 2) out << "\n"; else out << " end\n";
            }
        } out << "endtri\n\n";
    }
    if (g_Config.ProgressMode != 0) std::cout << std::endl;
    out.close(); std::cout << " -> Zapisano " << nodeCounter << " node'ow SCM." << std::endl;
}

// -----------------------------------------------------------
// ZAPIS E3D (Binarny) 
// Implementacja w oparciu o kwadraty kilometrowe (zgodnie ze specyfikacją wiki)
// -----------------------------------------------------------

void WritePaddedChunk(std::ofstream& out, const char* name, const std::vector<char>& data) {
    uint32_t size = 8 + data.size();
    uint32_t padding = (4 - (size % 4)) % 4;
    size += padding;
    out.write(name, 4);
    out.write(reinterpret_cast<const char*>(&size), 4);
    if (!data.empty()) out.write(data.data(), data.size());
    const char pad[3] = {0,0,0};
    if (padding > 0) out.write(pad, padding);
}

void SaveE3D(const std::string& filename, const std::vector<TerrainPoint>& points, const std::vector<TriangleSortInfo>& sortedTris) {
    std::cout << "[ZAPISYWANIE] Plik binarny E3D: " << filename << "..." << std::endl;
    
    std::ofstream out(filename, std::ios::binary);
    if (!out.is_open()) {
        std::cerr << "BLAD: Nie mozna utworzyc pliku " << filename << std::endl; return;
    }

    // 1. Grupowanie trójkątów w kwadraty kilometrowe (np. 500500)
    std::map<std::pair<int, int>, std::vector<TriangleSortInfo>> grid;
    for (const auto& t : sortedTris) {
        int kx = static_cast<int>(std::floor(t.cx / 1000.0f));
        int kz = static_cast<int>(std::floor(t.cz / 1000.0f));
        grid[{kx, kz}].push_back(t);
    }

    // 2. Budowa tabeli Materialow (TEX0)
    std::vector<char> texData;
    texData.push_back('\0'); // Indeks 0 jest uzywany jako "colored"
    std::string matName = "grass";
    texData.insert(texData.end(), matName.begin(), matName.end());
    texData.push_back('\0');

    // 3. Budowa struktur VNT0, NAM0, SUB0
    std::vector<char> namData;
    std::vector<float> vntData;
    std::vector<Sub0Entry> submodels;
    
    int nameIndexCounter = 0;

    for (auto it = grid.begin(); it != grid.end(); ++it) {
        const auto& cellPos = it->first;
        const auto& tris = it->second;

        // Generowanie nazwy kwadratu (np. 500500)
        int name_x = 500 + cellPos.first;
        int name_z = 500 + cellPos.second;
        char nameBuf[16];
        sprintf(nameBuf, "%03d%03d", name_x, name_z);
        std::string cellName(nameBuf);
        
        namData.insert(namData.end(), cellName.begin(), cellName.end());
        namData.push_back('\0');
        int currentNameIdx = nameIndexCounter++;

        // Budowa submodelu (rozmiar rygorystycznie 256 bajtów)
        Sub0Entry sub;
        std::memset(&sub, 0, sizeof(Sub0Entry)); // Wyzerowanie wszystkiego

        sub.next = -1; // Zostanie zaktualizowane za chwilę
        sub.child = -1;
        sub.type = 4; // GL_TRIANGLES
        sub.name_id = currentNameIdx;
        sub.anim_type = 0; // false
        sub.flags = 0x10;  // Bit 4 (16) - Render w cyklu nieprzezroczystych
        sub.matrix_id = -1;
        sub.first_vertex = vntData.size() / 8; // 8 floatów na wierzchołek
        sub.vertex_count = tris.size() * 3;
        sub.material_id = 1; // "grass" jest pod indeksem 1 w TEX0
        sub.min_darkness = 0.0f;
        sub.light_threshold = 0.0f;
        sub.diffuse[0] = 1.0f; sub.diffuse[1] = 1.0f; sub.diffuse[2] = 1.0f; sub.diffuse[3] = 1.0f;
        sub.max_vis_dist_sq = 64000000.0f; // 8000m x 8000m
        sub.min_vis_dist_sq = 0.0f;
        sub.index_count = 0; // Brak IDX
        sub.first_index = 0;
        sub.diffuse_mult = 1.0f;

        submodels.push_back(sub);

        // Pakowanie wierzchołków do gigantycznego bufora VNT
        for (const auto& t : tris) {
            size_t idx[] = {t.v1, t.v2, t.v3};
            for (int v = 0; v < 3; v++) {
                const auto& p = points[idx[v]];
                vntData.push_back(p.pos.x);
                vntData.push_back(p.pos.y);
                vntData.push_back(p.pos.z);
                vntData.push_back(p.normal.x);
                vntData.push_back(p.normal.y);
                vntData.push_back(p.normal.z);
                vntData.push_back(p.pos.x * 0.04f); // U
                vntData.push_back(p.pos.z * 0.04f); // V
            }
        }
    }

    // Aktualizacja powiązań (next) w submodelach
    for (size_t i = 0; i < submodels.size(); i++) {
        if (i < submodels.size() - 1) {
            submodels[i].next = i + 1;
            // Magia flag "OR bitami" wg wiki dla bezpieczeństwa silnika (bit 4 przesunięty do 24-31)
            submodels[i].flags |= 0x10000000; 
        }
    }

    // 4. Kalkulacja rozmiarów i zapis
    std::vector<char> subBytes(reinterpret_cast<char*>(submodels.data()), reinterpret_cast<char*>(submodels.data()) + submodels.size() * sizeof(Sub0Entry));
    std::vector<char> vntBytes(reinterpret_cast<char*>(vntData.data()), reinterpret_cast<char*>(vntData.data()) + vntData.size() * sizeof(float));

    // Symulacja zapisywania chunków do bufora (aby obliczyć główny rozmiar E3D0)
    uint32_t s_tex = 8 + texData.size() + ((4 - ((8 + texData.size()) % 4)) % 4);
    uint32_t s_nam = 8 + namData.size() + ((4 - ((8 + namData.size()) % 4)) % 4);
    uint32_t s_sub = 8 + subBytes.size(); // 256 jest podzielne przez 4, wiec padding = 0
    uint32_t s_vnt = 8 + vntBytes.size(); // float * ilość jest podzielne przez 4
    uint32_t totalSize = 8 + s_tex + s_nam + s_sub + s_vnt;

    // Główna Kromka E3D0
    out.write("E3D0", 4);
    out.write(reinterpret_cast<const char*>(&totalSize), 4);
    
    // Wnętrze E3D0
    WritePaddedChunk(out, "TEX0", texData);
    WritePaddedChunk(out, "NAM0", namData);
    WritePaddedChunk(out, "SUB0", subBytes);
    WritePaddedChunk(out, "VNT0", vntBytes);

    out.close();
    std::cout << " -> Zapisano " << submodels.size() << " kwadratow kilometrowych (Brak limitu nodes!)." << std::endl;
}

// -----------------------------------------------------------
// Główna funkcja - wczytywanie konfiguracji, odczyt danych, generowanie nasypów, przetwarzanie terenu i zapisywanie
// -----------------------------------------------------------

void PrintConfig() {
    std::cout << "\n--- ternAI konfiguracja ---" << std::endl;
    if (g_Config.OffsetsLoaded) std::cout << "  Offsety: OK (" << g_Config.OffsetEast << ", " << g_Config.OffsetNorth << ")" << std::endl;
    else std::cout << "  Offsety: BRAK" << std::endl;
    std::cout << "  Plik SCN:  " << g_Config.FileSCN << "\n  Kat. NMT1: " << g_Config.DirNMT1 << std::endl;
    
    std::cout << "  Eksport:   ";
    if (g_Config.ExportSCN) std::cout << "SCN (" << g_Config.FileOutputSCN << ") ";
    if (g_Config.ExportE3D) std::cout << "E3D (" << g_Config.FileOutputE3D << ") ";
    std::cout << std::endl;
}

void ParseArgs(int argc, char* argv[]) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-help" || arg == "/?") { PrintConfig(); exit(0); }
    }
}

int main(int argc, char* argv[]) {
    std::locale::global(std::locale("C"));
    LoadIniConfig("terenEU.ini");
    std::cout << "--- terenAI " << PROG_VERSION << " ===" << std::endl;
    ParseArgs(argc, argv);
    ParseSCNOffsets(g_Config.FileSCN);
    PrintConfig();
    
    if (!g_Config.ExportSCN && !g_Config.ExportE3D) {
        std::cout << "OSTRZEZENIE: Nie wybrano zadnego formatu eksportu w INI (ExportSCN=0 i ExportE3D=0). Koncze." << std::endl;
        return 0;
    }

    if (!g_Config.OffsetsLoaded) {
        std::cout << "Podaj Offset East (km): "; std::cin >> g_Config.OffsetEast;
        std::cout << "Podaj Offset North (km): "; std::cin >> g_Config.OffsetNorth;
        g_Config.OffsetEast *= 1000.0; g_Config.OffsetNorth *= 1000.0;
    }
    
    std::vector<TrackSegment> tracks;
    LoadTracksFromSCN(g_Config.FileSCN, tracks);
    if (tracks.empty()) { std::cerr << "BLAD: Brak torow!" << std::endl; return 1; }
    
    std::vector<TerrainPoint> points;
    LoadNMT1_Turbo(g_Config.DirNMT1, points);
    LoadNMT100_TXT(g_Config.FileNMT100, points);
    
    GenerateEmbankmentPoints(tracks, points);
    RemoveDuplicates(points);
    ProcessTerrain(points, tracks);

    std::vector<TriangleSortInfo> sortedTris;
    ProcessGeometry(points, sortedTris); 

    if (g_Config.ExportSCN) {
        SaveSCM(g_Config.FileOutputSCN, points, sortedTris);
    }
    
    if (g_Config.ExportE3D) {
        SaveE3D(g_Config.FileOutputE3D, points, sortedTris);
    }

    std::cout << "Gotowe." << std::endl;
    return 0;
}