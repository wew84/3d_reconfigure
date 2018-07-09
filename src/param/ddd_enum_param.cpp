#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "modernize-loop-convert"
#pragma ide diagnostic ignored "modernize-use-auto"
//
// Created by Noam Dori on 5/07/18.
//

#include <dddynamic_reconfigure/param/ddd_enum_param.h>

#include "dddynamic_reconfigure/param/ddd_enum_param.h"

map<string,pair<int,string> > fillGaps(map<string,int> old_map) {
    map<string,pair<int,string> > ret;
    for(map<string,int>::const_iterator it = old_map.begin(); it != old_map.end(); it++) {
        ret[it->first] = pair<int,string>(it->second,"");
    };
    return ret;
};

namespace dddynamic_reconfigure {

    bool DDDEnum::sameType(Value val) {
        return val.getType() == "int" || val.getType() == "string";
    }

    bool DDDEnum::sameValue(Value val) {
        if(val.getType() == "string" && dict_.find(val.toString())->second.first == val_) {
            return true;
        } else {
            return val.toInt() == val_;
        }
    }

    void DDDEnum::setValue(Value val) {
        if(val.getType() == "string" && dict_.find(val.toString()) != dict_.end()) {
            val_ = dict_.find(val.toString())->second.first;
        } else {
            val_ = val.toInt();
        }
    }

    DDDEnum::DDDEnum(const string &name, unsigned int level, const string &description,
            int def, const map<string, int> &dictionary) :
            DDDInt(name,level,description,def),
            dict_(fillGaps(dictionary)) {
        max_ = def;
        min_ = def;
        for(map<string,int>::const_iterator it = dictionary.begin(); it != dictionary.end(); it++) {
            if(it->second > max_) {max_ = it->second;}
            if(it->second < min_) {min_ = it->second;}
        };
    }

    DDDEnum::DDDEnum(const string &name, unsigned int level, const string &description,
            const string &def, const map<string, int> &dictionary) :
            DDDInt(name,level,description,dictionary.find(def)->second),
            dict_(fillGaps(dictionary)) {
        max_ = def_;
        min_ = def_;
        for(map<string,int>::const_iterator it = dictionary.begin(); it != dictionary.end(); it++) {
            if(it->second > max_) {max_ = it->second;}
            if(it->second < min_) {min_ = it->second;}
        };
    }

    #pragma clang diagnostic push
    #pragma clang diagnostic ignored "-Wdangling-field"
    DDDEnum::DDDEnum(const string &name, unsigned int level, const string &description, int def,
                   const pair<EnumMap, string> &dictionary) :
                   DDDInt(name,level,description,def),
                   dict_(dictionary.first) {
        max_ = def;
        min_ = def;
        for(EnumMap::const_iterator it = dict_.begin(); it != dict_.end(); it++) {
            if(it->second.first > max_) {max_ = it->second.first;}
            if(it->second.first < min_) {min_ = it->second.first;}
        };
    }

    DDDEnum::DDDEnum(const string &name, unsigned int level, const string &description, const string &def,
                   const pair<EnumMap,string> &dictionary) :
                   DDDInt(name,level,description,dictionary.first.find(def)->second.first),
                   dict_(dictionary.first) {
        max_ = def_;
        min_ = def_;
        for(EnumMap::const_iterator it = dict_.begin(); it != dict_.end(); it++) {
            if(it->second.first > max_) {max_ = it->second.first;}
            if(it->second.first < min_) {min_ = it->second.first;}
        };
    }

    bool DDDEnum::hasDefinition(string name) {
        return dict_.find(name) != dict_.end();
    }

    int DDDEnum::getDefinition(string name) {
        return dict_.find(name)->second.first;
    }

    void DDDEnum::removeDefinition(string name) {
        dict_.erase(name);
    }

    void DDDEnum::addDefinition(string name, int definition) {
        addDefinition(name,definition,"");
    }

    void DDDEnum::addDefinition(string name, int definition, string description) {
        dict_[name] = pair<int,string>(definition,description);
    }
}
#pragma clang diagnostic pop