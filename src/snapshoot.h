# pragma once
#include <cstdint>
#include <cstring>
#include <vector>
#include <array>
#include <fstream>
#include <string>

namespace snap {
using namespace std;
class Snapshoot {
    public:
    
    using Poly = vector<array<float, 2>>;

    Poly feasiblePoly;
    vector<Poly> polys;
    vector<vector<array<float, 2>>> frames;
    vector<vector<bool>> isLegal;

    Snapshoot() = default;
    void setPoly(const Poly& feasiblePoly, const vector<Poly>& polys) {
        this->feasiblePoly = feasiblePoly;
        this->polys = polys;
    }
    void addFrame(const vector<array<float, 2>> &frame,
                  const vector<bool> &legal) {
      // 最多记录 100000 帧
      if (100000 < frames.size()) return;
        
        frames.push_back(frame);
        isLegal.push_back(legal);
    }
    void write(const string& filename) {
        ofstream out(filename, ios::binary | ios::trunc);
        if (!out.is_open()) {
            return;
        }

        const auto writeU32 = [&](uint32_t value) {
            unsigned char bytes[4];
            bytes[0] = static_cast<unsigned char>(value & 0xFFu);
            bytes[1] = static_cast<unsigned char>((value >> 8) & 0xFFu);
            bytes[2] = static_cast<unsigned char>((value >> 16) & 0xFFu);
            bytes[3] = static_cast<unsigned char>((value >> 24) & 0xFFu);
            out.write(reinterpret_cast<const char*>(bytes), sizeof(bytes));
        };
        const auto writeF32 = [&](float value) {
            uint32_t bits = 0;
            static_assert(sizeof(bits) == sizeof(value), "float must be 32-bit");
            std::memcpy(&bits, &value, sizeof(bits));
            writeU32(bits);
        };
        const auto writeU8 = [&](uint8_t value) {
            out.write(reinterpret_cast<const char*>(&value), sizeof(value));
        };

        writeU32(static_cast<uint32_t>(feasiblePoly.size()));
        for (const auto& point : feasiblePoly) {
            writeF32(point[0]);
            writeF32(point[1]);
        }

        writeU32(static_cast<uint32_t>(polys.size()));
        for (const auto& poly : polys) {
            writeU32(static_cast<uint32_t>(poly.size()));
            for (const auto& point : poly) {
                writeF32(point[0]);
                writeF32(point[1]);
            }
        }

        writeU32(static_cast<uint32_t>(frames.size()));
        for (size_t frameIndex = 0; frameIndex < frames.size(); ++frameIndex) {
            const auto& frame = frames[frameIndex];
            const auto& legal = isLegal[frameIndex];

            for (const auto& point : frame) {
                writeF32(point[0]);
                writeF32(point[1]);
            }
            for (bool flag : legal) {
                writeU8(static_cast<uint8_t>(flag ? 1 : 0));
            }
        }

        out.close();
    }
};
static Snapshoot g_snapshoot;



}