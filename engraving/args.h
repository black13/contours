// args.h — simple key-value CLI argument parser
//
// Usage:
//   Args args(argc, argv);
//   int samples = args.getInt("--samples", 2000);
//   double lx = args.getDouble("--light-x", 1.0);
//   std::string out = args.getString("--output", "out.pdf");
//   bool hasPdf = args.hasFlag("--pdf");

#ifndef ENGRAVING_ARGS_H
#define ENGRAVING_ARGS_H

#include <string>
#include <cstdlib>
#include <map>

struct Args {
    std::map<std::string, std::string> kv;
    std::string program;

    Args(int argc, char** argv) {
        program = argv[0];
        for (int i = 1; i < argc; i++) {
            std::string key = argv[i];
            if (key.size() >= 2 && key[0] == '-' && key[1] == '-') {
                if (i + 1 < argc && argv[i+1][0] != '-') {
                    kv[key] = argv[++i];
                } else {
                    kv[key] = "1";  // flag
                }
            }
        }
    }

    bool hasFlag(const std::string& key) const {
        return kv.count(key) > 0;
    }

    std::string getString(const std::string& key, const std::string& def = "") const {
        auto it = kv.find(key);
        return it != kv.end() ? it->second : def;
    }

    int getInt(const std::string& key, int def = 0) const {
        auto it = kv.find(key);
        return it != kv.end() ? atoi(it->second.c_str()) : def;
    }

    double getDouble(const std::string& key, double def = 0.0) const {
        auto it = kv.find(key);
        return it != kv.end() ? atof(it->second.c_str()) : def;
    }
};

#endif // ENGRAVING_ARGS_H
