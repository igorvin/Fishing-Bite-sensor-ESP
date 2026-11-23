#pragma once
namespace bite {
// Keep the Settings virtual tick and explicit saves under one storage policy.
template<class FileDatabase> class ConfigDatabase : public FileDatabase {
    bool writable_=false;
public:
    using FileDatabase::FileDatabase;
    // GyverDBFile destructor calls its nonvirtual update(). Clear the dirty
    // flag before that destructor when persistence was never safe.
    ~ConfigDatabase(){if(!writable_)this->_update=false;}
    bool begin(){writable_=FileDatabase::begin();if(!writable_)this->_update=false;return writable_;}
    bool update(){if(!writable_){this->_update=false;return false;}return FileDatabase::update();}
    bool tick() override {if(!writable_){this->_update=false;return false;}return FileDatabase::tick();}
};
}
