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

    void DDDEnum::prepGroup(Group &group) {
        ParamDescription desc;
        desc.name  = name_;
        desc.level = level_;
        desc.description = desc_;
        desc.type = "int";
        desc.edit_method = makeEditMethod();
        group.parameters.push_back(desc);
    }

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
        val_ = lookup(val);
    }

    void DDDEnum::setDefault(Value val) {
        def_ = lookup(val);
    }

    void DDDEnum::setMax(Value val) {
        max_ = lookup(val);
    }

    void DDDEnum::setMin(Value val) {
        min_ = lookup(val);
    }

    int DDDEnum::lookup(Value val) {
        if(val.getType() == "string" && dict_.find(val.toString()) != dict_.end()) {
            return dict_.find(val.toString())->second.first;
        } else {
            return val.toInt();
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
        enum_description_ = dictionary.second;
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
        enum_description_ = dictionary.second;
    }

    string DDDEnum::makeEditMethod() {
        stringstream ret;
        ret << "{";
        {
            ret << "'enum_description': '" << enum_description_ << "', ";
            ret << "'enum': [";
            {
                EnumMap::const_iterator it = dict_.begin();
                ret << makeConst(it->first, it->second.first, it->second.second);
                for(it++; it != dict_.end(); it++) {
                    ret << ", " << makeConst(it->first, it->second.first, it->second.second);
                };
            }
            ret << "]";
        }
        ret << "}";
        return ret.str();
    }

    string DDDEnum::makeConst(string name, int value, string desc) {
        stringstream ret;
        ret << "{";
        {
            ret << "'srcline': 0, "; // the sole reason this is here is because dynamic placed it in its enum JSON.
            ret << "'description': '" << desc << "', ";
            ret << "'srcfile': '/does/this/really/matter.cfg', "; // the answer is no. This is useless.
            ret << "'cconsttype': 'const int', ";
            ret << "'value': '" << value << "', ";
            ret << "'ctype': 'int', ";
            ret << "'type': 'int', ";
            ret << "'name': '" << name << "'";
        }
        ret << "}";
        return ret.str();
    }

    bool DDDEnum::hasDefinition(string name) {
        return dict_.find(name) != dict_.end();
    }

    int DDDEnum::getDefinition(string name) {
        EnumMap::iterator it = dict_.find(name);
        if(dict_.end() == it) {
            return def_;
        }
        return it->second.first;
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

    DDPtr DDDEnum::copy() {
        return DDPtr(new DDDEnum(*this));
    }
}
#pragma clang diagnostic pop