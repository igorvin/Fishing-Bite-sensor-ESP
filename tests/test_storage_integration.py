#!/usr/bin/env python3
"""Run the installed GyverDBFile scheduler with fake platform/file IO.

Only platform includes are removed from the actual dependency header. Its begin,
update, tick and destructor bodies run unchanged, through the production guard.
Use --baseline to reproduce the original unguarded firmware behavior.
"""
import pathlib
import subprocess
import sys
import tempfile
import os

root = pathlib.Path(__file__).resolve().parents[1]
vendor = root / ".pio/libdeps/xiao_esp32c3/GyverDB/src/GyverDBFile.h"
source = vendor.read_text()
for line in ('#pragma once', '#include <Arduino.h>', '#include <FS.h>', '#include "GyverDB.h"'):
    source = source.replace(line, '')
shim = r'''
#include <cstdint>
#include <cstddef>
#include <string>
#include <cstdio>
uint32_t clockMs=0;
uint32_t millis(){return clockMs;}
namespace fs {struct FS;}
struct File {
    ~File(){}
    std::string* bytes=nullptr; bool readable=false;
    explicit operator bool()const{return bytes;}
    size_t size()const{return bytes?bytes->size():0;}
};
namespace fs {
struct FS {
    std::string bytes="corrupt original";bool present=true,readable=true;int writes=0;
    bool exists(const char*){return present;}
    File open(const char*,const char* mode){
        if(*mode=='r')return readable?File{&bytes,true}:File{};
        ++writes;present=true;bytes.clear();return {&bytes,false};
    }
};
}
class GyverDB {
protected: bool _update=false;
public:
    virtual bool tick(){return false;}
    void set(){_update=true;}
    bool readFrom(File& f,size_t){return *f.bytes=="valid";}
    bool writeTo(File& f){*f.bytes="saved";return true;}
};
'''
test = r'''
int main(){
    int failed=0;
    auto check=[&](bool ok,const char* msg){if(!ok){++failed;std::fprintf(stderr,"FAIL: %s\n",msg);}};
    for(bool readable:{false,true}){
        fs::FS storage;storage.readable=readable;auto original=storage.bytes;
        {
            TestDb db(&storage,"/config.db");
            check(!db.begin(),"unreadable/corrupt DB rejected");
            db.set(); // loadConfig defaults
            GyverDB* settings=&db;
            clockMs=1;settings->tick();
            db.set(); // UI setting and calibration result
            clockMs=12002;settings->tick();
            check(storage.bytes==original&&storage.writes==0,"real virtual auto-write preserves original bytes");
            db.set();check(!db.update(),"real explicit save blocked");
            db.set(); // destructor must not write either
        }
        check(storage.bytes==original&&storage.writes==0,"real destructor preserves original bytes");
    }
    fs::FS fresh;fresh.present=false;
    {TestDb db(&fresh,"/config.db");check(db.begin(),"absent DB created");db.set();check(db.update(),"new DB explicit save");
     db.set();clockMs=1;GyverDB* settings=&db;settings->tick();clockMs=12002;settings->tick();check(fresh.bytes=="saved"&&fresh.writes==3,"new DB auto save");}
    std::printf("actual GyverDBFile storage integration: %d failed\n",failed);
    return failed?1:0;
}
'''
alias = 'using TestDb=GyverDBFile;' if '--baseline' in sys.argv else '#include "ConfigDatabase.h"\nusing TestDb=bite::ConfigDatabase<GyverDBFile>;'
with tempfile.TemporaryDirectory(prefix='bite-storage-') as tmp:
    cpp = pathlib.Path(tmp) / 'storage.cpp'
    exe = pathlib.Path(tmp) / 'storage'
    cpp.write_text(shim + source + '\n' + alias + '\n' + test)
    subprocess.run([os.environ.get('CXX','c++'),'-std=c++17','-Wall','-Wextra','-Werror','-I'+str(root/'include'),str(cpp),'-o',str(exe)],check=True)
    sys.exit(subprocess.run([str(exe)]).returncode)
