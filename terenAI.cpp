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
#include <cstdint> // Dodane dla typow binarnych uint32_t

// -----------------------------------------------------------
// Plik delaunator.hpp musi być w katalogu projektu
// -----------------------------------------------------------
#include "delaunator.hpp" 

namespace fs = std::filesystem;

// -----------------------------------------------------------
// Konfiguracja i struktury danych
// -----------------------------------------------------------

const std::string PROG_VERSION = "v266 (E3D+SCM)";

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

    // Nazwy plików i katalogów
    std::string DirNMT1 = "NMT1";
    std::string FileNMT100 = "NMT100.txt";
    std::string FileSCN = "EXPORT.SCN";
    
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
    float minX, maxX, minZ, maxZ; 
    
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
// Pomocnicza klasa dla formatu E3D (Zapis binarny wg specyfikacji MaSzyny)
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
    
    // Wyrównanie do wielokrotności 4 bajtów
    void pad() { while (data.size() % 4 != 0) data.push_back(0); }
    
    // Zapisuje wielkość kromki do odpowiedniego miejsca
    void finalizeLength() {
        if (data.size() >= 8) {
            uint32_t size = static_cast<uint32_t>(data.size());
            std::memcpy(&data[4], &size, 4);
        }
    }
};

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
        if (file.read(buffer.data(), size)) {
            buffer[size] = '\0';
            return buffer;
        }
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
        Vector3 p = p1 * uuu; 
        p = p + c1 * (3 * uu * t);
        p = p + c2 * (3 * u * tt);
        p = p + p2 * ttt;
        return p;
    }

    static float DistanceSquared2D(const Vector3& a, const Vector3& b) {
        float dx = a.x - b.x; float dz = a.z - b.z;
        return dx * dx + dz * dz;
    }

    static float GetDistanceToSegment(const Vector3& p, const TrackSegment& seg, float& outTrackY) {
        float l2 = DistanceSquared2D(seg.p1, seg.p2);
        if (l2 == 0.0f) { outTrackY = seg.p1.y; return std::sqrt(DistanceSquared2D(p, seg.p1)); }
        float t = ((p.x - seg.p1.x) * (seg.p2.x - seg.p1.x) + (p.z - seg.p1.z) * (seg.p2.z - seg.p1.z)) / l2;
        t = std::max(0.0f, std::min(1.0f, t));
        Vector3 proj;
        proj.x = seg.p1.x + t * (seg.p2.x - seg.p1.x);
        proj.z = seg.p1.z + t * (seg.p2.z - seg.p1.z);
        outTrackY = Lerp(seg.p1.y, seg.p2.y, t);
        float dx = p.x - proj.x; float dz = p.z - proj.z;
        return std::sqrt(dx * dx + dz * dz);
    }

    static Vector3 GeoToSim(double geoX_North, double geoY_East, float height) {
        Vector3 v;
        v.x = static_cast<float>(g_Config.OffsetEast - geoY_East); 
        v.z = static_cast<float>(geoX_North - g_Config.OffsetNorth);
        v.y = height;
        return v;
    }
};

// -----------------------------------------------------------
// Siatki przestrzenne
// -----------------------------------------------------------

template <int CellSize>
class SpatialGrid {
    struct CellKey {
        int x, z;
        bool operator==(const CellKey& other) const { return x == other.x && z == other.z; }
    };
    struct KeyHasher {
        std::size_t operator()(const CellKey& k) const {
            return std::hash<int>()(k.x) ^ (std::hash<int>()(k.z) << 1);
        }
    };
    std::unordered_map<CellKey, std::vector<const TrackSegment*>, KeyHasher> grid;
    int GetIndex(float val) const { return static_cast<int>(std::floor(val / CellSize)); }

public:
    void Build(const std::vector<TrackSegment>& tracks) {
        grid.clear();
        for (const auto& t : tracks) {
            int minIx = GetIndex(t.minX); int maxIx = GetIndex(t.maxX);
            int minIz = GetIndex(t.minZ); int maxIz = GetIndex(t.maxZ);
            for (int x = minIx; x <= maxIx; ++x) {
                for (int z = minIz; z <= maxIz; ++z) {
                    grid[{x, z}].push_back(&t);
                }
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
    struct CellKey {
        int x, z;
        bool operator==(const CellKey& other) const { return x == other.x && z == other.z; }
    };
    struct KeyHasher {
        std::size_t operator()(const CellKey& k) const {
            return std::hash<int>()(k.x) ^ (std::hash<int>()(k.z) << 1);
        }
    };
    std::unordered_map<CellKey, std::vector<Vector3>, KeyHasher> grid;
    const int CellSize = 10; 

    int GetIndex(float val) const { return static_cast<int>(std::floor(val / CellSize)); }

public:
    void Add(const Vector3& p) {
        grid[{GetIndex(p.x), GetIndex(p.z)}].push_back(p);
    }

    bool HasNeighbor(const Vector3& p, float radius) const {
        int cx = GetIndex(p.x);
        int cz = GetIndex(p.z);
        float r2 = radius * radius;

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

// -----------------------------------------------------------
// Odczyt pliku .ini oraz offsetu z pliku SCN
// -----------------------------------------------------------

void LoadIniConfig(const std::string& filename) {
    if (!fs::exists(filename)) {
        std::cerr << "========================================================\n";
        std::cerr << " BLAD KRYTYCZNY: Brak pliku konfiguracyjnego '.ini'!\n";
        std::cerr << " Utworz plik '" << filename << "' w folderze z programem.\n";
        std::cerr << "========================================================\n";
        exit(1); // Wymóg posiadania INI - program zamknie się natychmiast
    }
    
    std::ifstream file(filename);
    std::string line;
    while (std::getline(file, line)) {
        line = Trim(line);
        if (line.empty() || line[0] == ';' || line[0] == '#' || line[0] == '[') continue;
        size_t eqPos = line.find('=');
        if (eqPos == std::string::npos) continue;
        
        std::string key = Trim(line.substr(0, eqPos));
        std::string val = Trim(line.substr(eqPos + 1));
        
        try {
            if (key == "FileSCN") g_Config.FileSCN = val;
            else if (key == "DirNMT1") g_Config.DirNMT1 = val;
            else if (key == "FileNMT100") g_Config.FileNMT100 = val;
            
            // Nowe tagi dla formatów Exportu
            else if (key == "ExportSCM") g_Config.ExportSCM = (val == "1" || val == "true" || val == "TRUE");
            else if (key == "OutputSCM" || key == "Output") g_Config.OutputSCM = val;
            else if (key == "ExportE3D") g_Config.ExportE3D = (val == "1" || val == "true" || val == "TRUE");
            else if (key == "OutputE3D") g_Config.OutputE3D = val;
            
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
            else if (key == "SwapNMT100Axes") g_Config.SwapNMT100Axes = (val == "1" || val == "true" || val == "TRUE");
        } catch (...) {}
    }
}

void PrintConfig() {
    std::cout << "\n========================================" << std::endl;
    std::cout << " Wczytana konfiguracja (z pliku .ini):" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << " [WEJSCIE]" << std::endl;
    std::cout << "  Plik torow SCN : " << g_Config.FileSCN << std::endl;
    std::cout << "  Katalog NMT1   : " << g_Config.DirNMT1 << std::endl;
    std::cout << "  Plik NMT100    : " << g_Config.FileNMT100 << std::endl;
    std::cout << "\n [EKSPORT]" << std::endl;
    std::cout << "  Zapis do SCM   : " << (g_Config.ExportSCM ? "TAK -> " + g_Config.OutputSCM : "NIE") << std::endl;
    std::cout << "  Zapis do E3D   : " << (g_Config.ExportE3D ? "TAK -> " + g_Config.OutputE3D : "NIE") << std::endl;
    std::cout << "\n [PARAMETRY TERENU]" << std::endl;
    std::cout << "  Limit NMT1     : " << g_Config.LimitNMT1 << " m" << std::endl;
    std::cout << "  Limit NMT100   : " << g_Config.LimitNMT100Max << " m" << std::endl;
    std::cout << "  Tolerancja Snap: " << g_Config.SnapDist << " m" << std::endl;
    std::cout << "  Dystans Smooth : " << g_Config.SmoothEnd << " m" << std::endl;
    std::cout << "  Max bok trojk. : " << g_Config.MaxTriangleEdge << " m" << std::endl;
    std::cout << "  Obnizenie toru : " << g_Config.TrackOffset << " m" << std::endl;
    std::cout << "========================================\n\n";
}

bool ParseSCNOffsets(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) return false;
    std::string line;
    for (int i = 0; i < 50; i++) {
        if (!std::getline(file, line)) break;
        if (line.find("//$g") != std::string::npos) {
            std::stringstream ss(line);
            std::string tag, sys; double kmEast, kmNorth;
            ss >> tag; 
            if (tag == "//$g") {
                ss >> sys; 
                if (ss >> kmEast >> kmNorth) {
                    g_Config.OffsetEast = kmEast * 1000.0;
                    g_Config.OffsetNorth = kmNorth * 1000.0;
                    g_Config.OffsetsLoaded = true;
                    return true;
                }
            }
        }
    }
    return false;
}

// -----------------------------------------------------------
// Odczyt danych z plików SCN, NMT1 i NMT100
// -----------------------------------------------------------

void LoadTracksFromSCN(const std::string& filename, std::vector<TrackSegment>& tracks) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "BLAD: Nie mozna otworzyc pliku SCN!" << std::endl;
        return;
    }
    std::string line;
    Vector3 p1, cv1, cv2, p2;
    bool foundP1 = false, foundCV1 = false, foundCV2 = false, foundP2 = false;
    bool inTrack = false;
    bool isTunnelTrack = false;
    bool isBridgeTrack = false;
    int totalSegments = 0;

    std::cout << "[WCZYTYWANIE] Tory (Normal + Switch)..." << std::endl;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        
        bool isNode = (line.find("node") != std::string::npos);
        bool isTrack = (line.find("track") != std::string::npos);
        
        if (isNode && isTrack) {
            bool isNormal = (line.find("normal") != std::string::npos);
            bool isSwitch = (line.find("switch") != std::string::npos);

            if (isNormal || isSwitch) {
                inTrack = true; 
                foundP1 = foundCV1 = foundCV2 = foundP2 = false; 
                isTunnelTrack = (line.find("tunnel") != std::string::npos);
                isBridgeTrack = (line.find("bridge") != std::string::npos);
            }
            continue;
        }

        if (inTrack) {
            if (line.find("endtrack") != std::string::npos) { inTrack = false; continue; }
            char firstChar = ' ';
            for(char c : line) { if(!isspace(c)) { firstChar = c; break; } }
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
                    if (segments < 2) segments = 2;
                    if (segments > 48) segments = 48;

                    for (int i = 1; i <= segments; i++) {
                        float t = (float)i / (float)segments;
                        Vector3 curr = MathUtils::CubicBezier(t, p1, cv1, cv2, p2);
                        Vector3 prev = (i == 1) ? p1 : MathUtils::CubicBezier((float)(i - 1) / segments, p1, cv1, cv2, p2);
                        TrackSegment tSeg; 
                        tSeg.p1 = prev; tSeg.p2 = curr; 
                        tSeg.isTunnel = isTunnelTrack;
                        tSeg.isBridge = isBridgeTrack;
                        tSeg.CalcBounds();
                        tracks.push_back(tSeg);
                        totalSegments++;
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
    for (const auto& entry : fs::directory_iterator(folderPath)) {
        if (entry.path().extension() == ".asc") ascFiles.push_back(entry.path());
    }
    if (ascFiles.empty()) return;

    unsigned int threadsCount = GetThreadCount();
    if (threadsCount > 2) threadsCount -= 2; else threadsCount = 1;

    std::cout << "[WCZYTYWANIE] NMT1 (" << ascFiles.size() << " plikow)..." << std::endl;
    std::atomic<size_t> filesProcessed(0);
    std::vector<std::vector<TerrainPoint>> threadBuffers(threadsCount);

    auto worker = [&](int threadIdx, size_t startIdx, size_t endIdx) {
        threadBuffers[threadIdx].reserve(500000); 
        for (size_t i = startIdx; i < endIdx; ++i) {
            auto buffer = FastParser::ReadFileToBuffer(ascFiles[i].string());
            if (buffer.empty()) continue;
            char* ptr = buffer.data();
            int ncols = 0, nrows = 0; double xll = 0, yll = 0, cellsize = 0, nodata = -9999;
            for(int h=0; h<6; h++) {
                ptr = FastParser::SkipWhitespace(ptr); while(*ptr && !isspace(*ptr)) ptr++;
                ptr = FastParser::SkipWhitespace(ptr); 
                if(h==0) ncols = std::atoi(ptr);
                else if(h==1) nrows = std::atoi(ptr);
                else if(h==2) xll = std::strtod(ptr, &ptr);
                else if(h==3) yll = std::strtod(ptr, &ptr);
                else if(h==4) cellsize = std::strtod(ptr, &ptr);
                else if(h==5) nodata = std::strtod(ptr, &ptr);
                if(h < 5) while(*ptr && *ptr != '\n') ptr++;
            }
            for (int r = 0; r < nrows; r++) {
                double currentGeoX = (yll + nrows * cellsize) - (r * cellsize);
                if (r % 10 != 0) {
                    for (int c = 0; c < ncols; c++) {
                        ptr = FastParser::SkipWhitespace(ptr); while (*ptr && !isspace(*ptr)) ptr++;
                    }
                } else {
                    for (int c = 0; c < ncols; c++) {
                        ptr = FastParser::SkipWhitespace(ptr);
                        if (c % 10 == 0) {
                            char* endPtr;
                            float zVal = std::strtof(ptr, &endPtr);
                            ptr = endPtr;
                            if (std::abs(zVal - nodata) > 0.1f) {
                                double currentGeoY = xll + (c * cellsize);
                                threadBuffers[threadIdx].push_back({MathUtils::GeoToSim(currentGeoX, currentGeoY, zVal), true, true, false});
                            }
                        } else {
                            while (*ptr && !isspace(*ptr)) ptr++;
                        }
                    }
                }
            }
            filesProcessed++;
        }
    };

    std::vector<std::thread> workers;
    size_t batchSize = (ascFiles.size() + threadsCount - 1) / threadsCount;
    for (unsigned int t = 0; t < threadsCount; ++t) {
        size_t start = t * batchSize;
        size_t end = std::min(start + batchSize, ascFiles.size());
        if (start < end) workers.emplace_back(worker, t, start, end);
    }
    while (filesProcessed < ascFiles.size()) {
        ShowProgress(filesProcessed, ascFiles.size(), "[WCZYTYWANIE] NMT1");
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    if (g_Config.ProgressMode != 0) std::cout << std::endl;

    for (auto& w : workers) w.join();
    for (auto& buf : threadBuffers) outPoints.insert(outPoints.end(), buf.begin(), buf.end());
    std::cout << " -> NMT1: " << outPoints.size() << " punktow." << std::endl;
}

void LoadNMT100_TXT(const std::string& filename, std::vector<TerrainPoint>& points) {
    if (!fs::exists(filename)) return;
    std::cout << "[WCZYTYWANIE] NMT100..." << std::endl;
    std::ifstream file(filename);
    if (!file.is_open()) return;
    double gX, gY, gH; 
    while (file >> gY >> gX >> gH) { 
        Vector3 pos;
        if (g_Config.SwapNMT100Axes) pos = MathUtils::GeoToSim(gX, gY, (float)gH); 
        else pos = MathUtils::GeoToSim(gY, gX, (float)gH);
        points.push_back({pos, false, true, false});
    }
}

// -----------------------------------------------------------
// Generowanie punktów nasypów wzdłuż torów
// -----------------------------------------------------------

void GenerateEmbankmentPoints(const std::vector<TrackSegment>& tracks, std::vector<TerrainPoint>& points) {
    std::cout << "[GENEROWANIE] Punkty nasypow..." << std::endl;
    
    PointGrid pGrid;
    for (const auto& p : points) pGrid.Add(p.pos);

    std::vector<TerrainPoint> newPoints;
    const float OFFSET_Y = g_Config.TrackOffset;
    const float OFFSET_XZ = g_Config.EmbankmentWidth;
    const float MIN_DIST = g_Config.MinPointDist;

    size_t processed = 0;
    size_t total = tracks.size();

    for (const auto& t : tracks) {
        processed++;
        if (processed % 1000 == 0) ShowProgress(processed, total, "[GENEROWANIE] Nasypy");

        if (t.isTunnel || t.isBridge) continue;

        Vector3 dir = t.p2 - t.p1;
        float len = dir.Length();
        if (len < 0.1f) continue;

        Vector3 dirNorm = dir.Normalized();
        Vector3 perp = {-dirNorm.z, 0.0f, dirNorm.x};

        float step = g_Config.EmbankmentStep;
        
        for (float d = 0.0f; d < len - 0.01f; d += step) {
            
            Vector3 centerPos = t.p1 + dirNorm * d;
            float trackHeight = MathUtils::Lerp(t.p1.y, t.p2.y, d/len);
            float terrainHeight = trackHeight - OFFSET_Y;

            Vector3 leftPos = centerPos + perp * OFFSET_XZ;
            leftPos.y = terrainHeight;

            Vector3 rightPos = centerPos - perp * OFFSET_XZ;
            rightPos.y = terrainHeight;

            if (!pGrid.HasNeighbor(leftPos, MIN_DIST)) {
                newPoints.push_back({leftPos, true, true, true}); 
                pGrid.Add(leftPos);
            }

            if (!pGrid.HasNeighbor(rightPos, MIN_DIST)) {
                newPoints.push_back({rightPos, true, true, true});
                pGrid.Add(rightPos);
            }
        }
    }
    if(g_Config.ProgressMode != 0) std::cout << std::endl;
    
    std::cout << " -> Dodano " << newPoints.size() << " punktow kotwiczacych." << std::endl;
    points.insert(points.end(), newPoints.begin(), newPoints.end());
}

// -----------------------------------------------------------
// Przetwarzanie terenu 
// ----------------------------------------------------------

void ProcessTerrain(std::vector<TerrainPoint>& points, const std::vector<TrackSegment>& tracks) {
    std::cout << "[OBLICZANIE] Budowa indeksow przestrzennych..." << std::endl;
    SpatialGrid<150> fineGrid; fineGrid.Build(tracks);
    SpatialGrid<4000> wideGrid; wideGrid.Build(tracks);

    const float LIMIT_NMT1 = g_Config.LimitNMT1;
    const float LIMIT_NMT100_MAX = g_Config.LimitNMT100Max;
    const float SNAP_DIST = g_Config.SnapDist; 
    const float SMOOTH_END = g_Config.SmoothEnd;
    const float TRACK_OFFSET = g_Config.TrackOffset;

    unsigned int threads = GetThreadCount();
    if (threads > 1) threads -= 1;

    std::cout << "[FILTROWANIE] Przetwarzanie terenu (" << threads << " watkow)..." << std::endl;
    std::vector<std::thread> workers;
    std::atomic<size_t> processed(0);
    size_t total = points.size();
    size_t chunk = (total + threads - 1) / threads;

    auto workerTask = [&](size_t start, size_t end) {
        std::vector<const TrackSegment*> nearbyFine; nearbyFine.reserve(200);
        std::vector<const TrackSegment*> nearbyWide; nearbyWide.reserve(200);
        
        for (size_t i = start; i < end; ++i) {
            TerrainPoint& pt = points[i];
            
            if (pt.isFixed) {
                pt.isValid = true;
                processed++;
                continue; 
            }

            fineGrid.GetNearby(pt.pos, nearbyFine);
            
            float absMinDist = 1e9f; 
            float sumWeightedHeights = 0.0f;
            float sumWeights = 0.0f;
            
            for (const auto* trk : nearbyFine) {
                float trkY; 
                float d = MathUtils::GetDistanceToSegment(pt.pos, *trk, trkY);
                
                if (d < absMinDist) absMinDist = d;

                if (trk->isTunnel || trk->isBridge) continue;

                if (d <= SMOOTH_END) {
                    float weight = (1.0f - (d / SMOOTH_END));
                    weight = weight * weight; 
                    float targetForThisTrack = trkY - TRACK_OFFSET; 
                    sumWeightedHeights += targetForThisTrack * weight;
                    sumWeights += weight;
                }
            }

            if (absMinDist <= SMOOTH_END && sumWeights > 0.0001f) {
                float weightedTargetH = sumWeightedHeights / sumWeights;
                if (absMinDist <= SNAP_DIST) {
                    pt.pos.y = weightedTargetH; 
                } 
                else {
                    float blendFactor = (absMinDist - SNAP_DIST) / (SMOOTH_END - SNAP_DIST);
                    pt.pos.y = MathUtils::Lerp(weightedTargetH, pt.pos.y, blendFactor);
                }
            }

            if (pt.isNMT1) {
                if (absMinDist <= LIMIT_NMT1) pt.isValid = true;
                else pt.isValid = false;
            } 
            else { 
                if (absMinDist <= LIMIT_NMT1) pt.isValid = false;
                else {
                    if (absMinDist <= LIMIT_NMT100_MAX) pt.isValid = true;
                    else {
                        nearbyWide.clear();
                        wideGrid.GetNearby(pt.pos, nearbyWide);
                        bool inRange = false;
                        for (const auto* trk : nearbyWide) {
                            float dummyH;
                            if (MathUtils::GetDistanceToSegment(pt.pos, *trk, dummyH) <= LIMIT_NMT100_MAX) {
                                inRange = true; break;
                            }
                        }
                        pt.isValid = inRange;
                    }
                }
            }
            processed++;
        }
    };

    for (unsigned int i = 0; i < threads; ++i) {
        size_t start = i * chunk;
        size_t end = std::min(start + chunk, total);
        workers.emplace_back(workerTask, start, end);
    }

    while(processed < total) {
        ShowProgress(processed, total, "[FILTROWANIE] Postep");
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
    if (g_Config.ProgressMode != 0) std::cout << std::endl;

    for (auto& w : workers) w.join();
    auto it = std::remove_if(points.begin(), points.end(), [](const TerrainPoint& p){ return !p.isValid; });
    points.erase(it, points.end());
}

void RemoveDuplicates(std::vector<TerrainPoint>& points) {
    std::cout << "[OBLICZANIE] Usuwanie duplikatow..." << std::endl;
    std::sort(points.begin(), points.end(), [](const TerrainPoint& a, const TerrainPoint& b) {
        if (std::abs(a.pos.x - b.pos.x) > 0.01f) return a.pos.x < b.pos.x;
        return a.pos.z < b.pos.z;
    });
    const float MERGE_DIST = g_Config.MergeClosePoints;
    auto last = std::unique(points.begin(), points.end(), [&](const TerrainPoint& a, const TerrainPoint& b) {
        return std::abs(a.pos.x - b.pos.x) < MERGE_DIST && std::abs(a.pos.z - b.pos.z) < MERGE_DIST;
    });
    points.erase(last, points.end());
}

// -----------------------------------------------------------
// EKSPORT TERENU (SCM ORAZ E3D)
// -----------------------------------------------------------

void ExportTerrain(std::vector<TerrainPoint>& points) {
    std::cout << "[GENEROWANIE] Triangulacja..." << std::endl;
    std::vector<double> coords; coords.reserve(points.size() * 2);
    for (const auto& p : points) { coords.push_back(p.pos.x); coords.push_back(p.pos.z); }
    delaunator::Delaunator d(coords);

    std::cout << "[GENEROWANIE] Obliczanie oswietlenia (Smooth Shading)..." << std::endl;
    for(auto& p : points) p.normal = {0,0,0};

    const float MAX_EDGE = g_Config.MaxTriangleEdge;
    std::vector<TriangleSortInfo> sortedTris;
    sortedTris.reserve(d.triangles.size() / 3);

    for (size_t i = 0; i < d.triangles.size(); i += 3) {
        size_t v1 = d.triangles[i]; size_t v2 = d.triangles[i+1]; size_t v3 = d.triangles[i+2];
        Vector3 p1 = points[v1].pos; Vector3 p2 = points[v2].pos; Vector3 p3 = points[v3].pos;
        
        float d1 = std::sqrt(std::pow(p1.x-p2.x, 2) + std::pow(p1.z-p2.z, 2));
        float d2 = std::sqrt(std::pow(p2.x-p3.x, 2) + std::pow(p2.z-p3.z, 2));
        float d3 = std::sqrt(std::pow(p3.x-p1.x, 2) + std::pow(p3.z-p1.z, 2));
        if (d1 > MAX_EDGE || d2 > MAX_EDGE || d3 > MAX_EDGE) continue;

        Vector3 U = p2 - p1;
        Vector3 V = p3 - p1;
        Vector3 FaceNormal = U.Cross(V); 

        points[v1].normal += FaceNormal;
        points[v2].normal += FaceNormal;
        points[v3].normal += FaceNormal;

        TriangleSortInfo t; t.v1 = v1; t.v2 = v2; t.v3 = v3;
        t.cx = (p1.x + p2.x + p3.x) / 3.0f; t.cz = (p1.z + p2.z + p3.z) / 3.0f;
        sortedTris.push_back(t);
    }

    for(auto& p : points) p.normal = p.normal.Normalized();

    std::cout << "[GENEROWANIE] Sortowanie trojkatow..." << std::endl;
    std::sort(sortedTris.begin(), sortedTris.end(), [](const TriangleSortInfo& a, const TriangleSortInfo& b) {
        int za = (int)(a.cz / 200.0f); int zb = (int)(b.cz / 200.0f);
        if (za != zb) return za < zb; return a.cx < b.cx;
    });

    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);

    // =====================================
    // EKSPORT: Format .scm (Tekstowy)
    // =====================================
    if (g_Config.ExportSCM) {
        std::cout << "[EKSPORT SCM] Plik: " << g_Config.OutputSCM << "..." << std::endl;
        std::ofstream out(g_Config.OutputSCM);
        out.imbue(std::locale("C"));
        out << std::fixed << std::setprecision(2);

        out << "// Generated by TerenAI " << PROG_VERSION << "\n";
        out << "// Date: " << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S") << "\n";
        out << "// Offset: East=" << g_Config.OffsetEast << ", North=" << g_Config.OffsetNorth << "\n\n";

        const size_t TRIANGLES_PER_NODE = 4;
        size_t nodeCounter = 0;
        size_t written = 0;
        size_t totalTris = sortedTris.size();

        for (size_t i = 0; i < sortedTris.size(); i += TRIANGLES_PER_NODE) {
            written += TRIANGLES_PER_NODE;
            if (written % 10000 == 0) ShowProgress(written, totalTris, "[EKSPORT SCM] Zapis");

            out << "node -1 0 teren_" << nodeCounter++ << " triangles grass\n";
            size_t batchEnd = std::min(i + TRIANGLES_PER_NODE, sortedTris.size());
            
            for (size_t k = i; k < batchEnd; k++) {
                const auto& t = sortedTris[k];
                size_t idx[] = {t.v1, t.v2, t.v3};
                
                for (int v = 0; v < 3; v++) {
                    const auto& p = points[idx[v]];
                    float u_tex = p.pos.x * 0.04f; 
                    float v_tex = p.pos.z * 0.04f;
                    
                    out << p.pos.x << " " << p.pos.y << " " << p.pos.z << " " 
                        << p.normal.x << " " << p.normal.y << " " << p.normal.z << " " 
                        << u_tex << " " << v_tex;
                    
                    if (k == batchEnd - 1 && v == 2) out << "\n"; else out << " end\n";
                }
            }
            out << "endtri\n\n";
        }
        if (g_Config.ProgressMode != 0) std::cout << std::endl;
        out.close();
        std::cout << " -> Zapisano " << nodeCounter << " wezlow w SCM." << std::endl;
    }

    // =====================================
    // EKSPORT: Format .e3d (Binarny)
    // =====================================
    if (g_Config.ExportE3D) {
        std::cout << "[EKSPORT E3D] Plik: " << g_Config.OutputE3D << "..." << std::endl;

        // --- KROMKA TEX0 ---
        E3DChunkWriter tex0;
        tex0.writeID("TEX0");
        tex0.writeU32(0); // placeholder dla wielkości
        tex0.data.push_back(0); // Element 0 nieużywany (wymóg dokumentacji)
        std::string textureName = "grass";
        tex0.data.insert(tex0.data.end(), textureName.begin(), textureName.end());
        tex0.data.push_back(0); // Null-terminator dla "grass" (teraz grass ma indeks 1)
        tex0.pad();
        tex0.finalizeLength();

        // --- KROMKA NAM0 ---
        E3DChunkWriter nam0;
        nam0.writeID("NAM0");
        nam0.writeU32(0);
        std::string submodelName = "teren";
        nam0.data.insert(nam0.data.end(), submodelName.begin(), submodelName.end());
        nam0.data.push_back(0); // Null-terminator (teren ma indeks 0)
        nam0.pad();
        nam0.finalizeLength();

        // --- KROMKA VNT0 ---
        E3DChunkWriter vnt0;
        vnt0.writeID("VNT0");
        vnt0.writeU32(0);
        for (const auto& p : points) {
            vnt0.writeF32(p.pos.x);
            vnt0.writeF32(p.pos.y);
            vnt0.writeF32(p.pos.z);
            vnt0.writeF32(p.normal.x);
            vnt0.writeF32(p.normal.y);
            vnt0.writeF32(p.normal.z);
            vnt0.writeF32(p.pos.x * 0.04f); // U
            vnt0.writeF32(p.pos.z * 0.04f); // V
        }
        vnt0.pad();
        vnt0.finalizeLength();

        // --- KROMKA IDX4 ---
        E3DChunkWriter idx4;
        idx4.writeID("IDX4");
        idx4.writeU32(0);
        for (const auto& t : sortedTris) {
            idx4.writeU32(static_cast<uint32_t>(t.v1));
            idx4.writeU32(static_cast<uint32_t>(t.v2));
            idx4.writeU32(static_cast<uint32_t>(t.v3));
        }
        idx4.pad();
        idx4.finalizeLength();

        // --- KROMKA SUB0 ---
        E3DChunkWriter sub0;
        sub0.writeID("SUB0");
        sub0.writeU32(0); // 256 bajtow danych + 8 header = 264. To wypełni finalizeLength()
        
        // Zapisywanie 256-bajtowej struktury dla jednego ogromnego submodelu
        sub0.writeI32(-1); // 0: Następny submodel (-1: brak)
        sub0.writeI32(-1); // 4: Potomek (-1: brak)
        sub0.writeI32(4);  // 8: Typ (4 = GL_TRIANGLES)
        sub0.writeI32(0);  // 12: Nr nazwy (0 = wzkazuje na "teren" w NAM0)
        sub0.writeI32(0);  // 16: Animacja (0 = false/nieruchomy)
        sub0.writeI32(16); // 20: Flagi (Bit 4 ustawiony = render opaque)
        sub0.writeI32(-1); // 24: Macierz przekształcenia (-1 = jednostkowa)
        sub0.writeI32(static_cast<int32_t>(points.size())); // 28: Ilość wierzchołków
        sub0.writeI32(0);  // 32: Pierwszy wierzchołek w VNT0
        sub0.writeI32(1);  // 36: Numer materiału (1 = "grass" w TEX0)
        sub0.writeF32(0.0f); // 40: Próg jasności
        sub0.writeF32(0.0f); // 44: Próg zapalenia światła
        
        // 48: Kolor Ambient (16 bajtów)
        sub0.writeF32(0); sub0.writeF32(0); sub0.writeF32(0); sub0.writeF32(0);
        // 64: Kolor Diffuse (16 bajtów)
        sub0.writeF32(1.0f); sub0.writeF32(1.0f); sub0.writeF32(1.0f); sub0.writeF32(1.0f);
        // 80: Kolor Specular (16 bajtów)
        sub0.writeF32(0); sub0.writeF32(0); sub0.writeF32(0); sub0.writeF32(0);
        // 96: Kolor Emisji (16 bajtów)
        sub0.writeF32(0); sub0.writeF32(0); sub0.writeF32(0); sub0.writeF32(0);
        
        sub0.writeF32(1.0f); // 112: Rozmiar linii
        sub0.writeF32(100000000.0f); // 116: Kwadrat max. odległości widoczności
        sub0.writeF32(0.0f); // 120: Kwadrat min. odległości widoczności
        
        // 124: Parametry światła (8 floatów = 32 bajty) - Zera
        sub0.writeZeroes(32);
        
        // 156: Ilość indeksów trójkątów
        sub0.writeI32(static_cast<int32_t>(sortedTris.size() * 3));
        // 160: Pierwszy indeks
        sub0.writeI32(0);
        // 164: Mnożnik diffuse
        sub0.writeF32(1.0f);
        
        // 168 do 255: Zmienne robocze - zera (88 bajtów)
        sub0.writeZeroes(88);
        
        sub0.finalizeLength();

        // --- KROMKA GŁÓWNA E3D0 ---
        E3DChunkWriter e3d0;
        e3d0.writeID("E3D0");
        
        uint32_t totalSize = 8 + 
                             static_cast<uint32_t>(sub0.data.size() + 
                                                   vnt0.data.size() + 
                                                   idx4.data.size() + 
                                                   tex0.data.size() + 
                                                   nam0.data.size());
        
        e3d0.writeU32(totalSize);
        e3d0.data.insert(e3d0.data.end(), sub0.data.begin(), sub0.data.end());
        e3d0.data.insert(e3d0.data.end(), vnt0.data.begin(), vnt0.data.end());
        e3d0.data.insert(e3d0.data.end(), idx4.data.begin(), idx4.data.end());
        e3d0.data.insert(e3d0.data.end(), tex0.data.begin(), tex0.data.end());
        e3d0.data.insert(e3d0.data.end(), nam0.data.begin(), nam0.data.end());

        // Zapis na dysk
        std::ofstream file(g_Config.OutputE3D, std::ios::binary);
        file.write(reinterpret_cast<const char*>(e3d0.data.data()), e3d0.data.size());
        file.close();

        std::cout << " -> Zapisano binarny plik E3D. Rozmiar: " << std::fixed << std::setprecision(1) << (e3d0.data.size() / 1024.0 / 1024.0) << " MB." << std::endl;
    }
}

// -----------------------------------------------------------
// Main
// -----------------------------------------------------------

void ParseArgs(int argc, char* argv[]) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-help" || arg == "/?") { PrintConfig(); exit(0); }
    }
}

int main(int argc, char* argv[]) {
    std::locale::global(std::locale("C"));
    std::cout << "--- terenAI " << PROG_VERSION << " --- \n";
    
    LoadIniConfig("terenAI.ini");
    ParseArgs(argc, argv);
    ParseSCNOffsets(g_Config.FileSCN);
    PrintConfig();
    
    if (!g_Config.ExportSCM && !g_Config.ExportE3D) {
        std::cerr << "BLAD: Wszystkie opcje eksportu w terenAI.ini sa wylaczone (ExportSCM i ExportE3D).\n";
        return 1;
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
    
    // Główne wywołanie nowej funkcji eksportującej
    ExportTerrain(points);
    
    std::cout << "Gotowe." << std::endl;
    return 0;
}