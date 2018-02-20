// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <ActionManager_IDL.h>
#include <yarp/os/idl/WireTypes.h>



class ActionManager_IDL_play : public yarp::os::Portable {
public:
  std::string name;
  double speed;
  bool sync;
  bool _return;
  void init(const std::string& name, const double speed, const bool sync);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class ActionManager_IDL_playFile : public yarp::os::Portable {
public:
  std::string filename;
  double speed;
  bool sync;
  bool _return;
  void init(const std::string& filename, const double speed, const bool sync);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class ActionManager_IDL_idleParts : public yarp::os::Portable {
public:
  std::string parts;
  bool _return;
  void init(const std::string& parts);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class ActionManager_IDL_activeParts : public yarp::os::Portable {
public:
  std::string parts;
  bool _return;
  void init(const std::string& parts);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class ActionManager_IDL_record : public yarp::os::Portable {
public:
  std::string name;
  std::string parts;
  double timeout;
  bool _return;
  void init(const std::string& name, const std::string& parts, const double timeout);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class ActionManager_IDL_stopRecording : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class ActionManager_IDL_stopPlaying : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class ActionManager_IDL_getAvailableActions : public yarp::os::Portable {
public:
  std::vector<std::string>  _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class ActionManager_IDL_setRecordingRate : public yarp::os::Portable {
public:
  int16_t rate_ms;
  void init(const int16_t rate_ms);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

bool ActionManager_IDL_play::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(4)) return false;
  if (!writer.writeTag("play",1,1)) return false;
  if (!writer.writeString(name)) return false;
  if (!writer.writeDouble(speed)) return false;
  if (!writer.writeBool(sync)) return false;
  return true;
}

bool ActionManager_IDL_play::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void ActionManager_IDL_play::init(const std::string& name, const double speed, const bool sync) {
  _return = false;
  this->name = name;
  this->speed = speed;
  this->sync = sync;
}

bool ActionManager_IDL_playFile::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(4)) return false;
  if (!writer.writeTag("playFile",1,1)) return false;
  if (!writer.writeString(filename)) return false;
  if (!writer.writeDouble(speed)) return false;
  if (!writer.writeBool(sync)) return false;
  return true;
}

bool ActionManager_IDL_playFile::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void ActionManager_IDL_playFile::init(const std::string& filename, const double speed, const bool sync) {
  _return = false;
  this->filename = filename;
  this->speed = speed;
  this->sync = sync;
}

bool ActionManager_IDL_idleParts::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("idleParts",1,1)) return false;
  if (!writer.writeString(parts)) return false;
  return true;
}

bool ActionManager_IDL_idleParts::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void ActionManager_IDL_idleParts::init(const std::string& parts) {
  _return = false;
  this->parts = parts;
}

bool ActionManager_IDL_activeParts::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("activeParts",1,1)) return false;
  if (!writer.writeString(parts)) return false;
  return true;
}

bool ActionManager_IDL_activeParts::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void ActionManager_IDL_activeParts::init(const std::string& parts) {
  _return = false;
  this->parts = parts;
}

bool ActionManager_IDL_record::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(4)) return false;
  if (!writer.writeTag("record",1,1)) return false;
  if (!writer.writeString(name)) return false;
  if (!writer.writeString(parts)) return false;
  if (!writer.writeDouble(timeout)) return false;
  return true;
}

bool ActionManager_IDL_record::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void ActionManager_IDL_record::init(const std::string& name, const std::string& parts, const double timeout) {
  _return = false;
  this->name = name;
  this->parts = parts;
  this->timeout = timeout;
}

bool ActionManager_IDL_stopRecording::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("stopRecording",1,1)) return false;
  return true;
}

bool ActionManager_IDL_stopRecording::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void ActionManager_IDL_stopRecording::init() {
  _return = false;
}

bool ActionManager_IDL_stopPlaying::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("stopPlaying",1,1)) return false;
  return true;
}

bool ActionManager_IDL_stopPlaying::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void ActionManager_IDL_stopPlaying::init() {
  _return = false;
}

bool ActionManager_IDL_getAvailableActions::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("getAvailableActions",1,1)) return false;
  return true;
}

bool ActionManager_IDL_getAvailableActions::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  {
    _return.clear();
    uint32_t _size0;
    yarp::os::idl::WireState _etype3;
    reader.readListBegin(_etype3, _size0);
    _return.resize(_size0);
    uint32_t _i4;
    for (_i4 = 0; _i4 < _size0; ++_i4)
    {
      if (!reader.readString(_return[_i4])) {
        reader.fail();
        return false;
      }
    }
    reader.readListEnd();
  }
  return true;
}

void ActionManager_IDL_getAvailableActions::init() {
}

bool ActionManager_IDL_setRecordingRate::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("setRecordingRate",1,1)) return false;
  if (!writer.writeI16(rate_ms)) return false;
  return true;
}

bool ActionManager_IDL_setRecordingRate::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  return true;
}

void ActionManager_IDL_setRecordingRate::init(const int16_t rate_ms) {
  this->rate_ms = rate_ms;
}

ActionManager_IDL::ActionManager_IDL() {
  yarp().setOwner(*this);
}
bool ActionManager_IDL::play(const std::string& name, const double speed, const bool sync) {
  bool _return = false;
  ActionManager_IDL_play helper;
  helper.init(name,speed,sync);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool ActionManager_IDL::play(const std::string& name, const double speed, const bool sync)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool ActionManager_IDL::playFile(const std::string& filename, const double speed, const bool sync) {
  bool _return = false;
  ActionManager_IDL_playFile helper;
  helper.init(filename,speed,sync);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool ActionManager_IDL::playFile(const std::string& filename, const double speed, const bool sync)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool ActionManager_IDL::idleParts(const std::string& parts) {
  bool _return = false;
  ActionManager_IDL_idleParts helper;
  helper.init(parts);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool ActionManager_IDL::idleParts(const std::string& parts)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool ActionManager_IDL::activeParts(const std::string& parts) {
  bool _return = false;
  ActionManager_IDL_activeParts helper;
  helper.init(parts);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool ActionManager_IDL::activeParts(const std::string& parts)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool ActionManager_IDL::record(const std::string& name, const std::string& parts, const double timeout) {
  bool _return = false;
  ActionManager_IDL_record helper;
  helper.init(name,parts,timeout);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool ActionManager_IDL::record(const std::string& name, const std::string& parts, const double timeout)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool ActionManager_IDL::stopRecording() {
  bool _return = false;
  ActionManager_IDL_stopRecording helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool ActionManager_IDL::stopRecording()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool ActionManager_IDL::stopPlaying() {
  bool _return = false;
  ActionManager_IDL_stopPlaying helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool ActionManager_IDL::stopPlaying()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::vector<std::string>  ActionManager_IDL::getAvailableActions() {
  std::vector<std::string>  _return;
  ActionManager_IDL_getAvailableActions helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::vector<std::string>  ActionManager_IDL::getAvailableActions()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
void ActionManager_IDL::setRecordingRate(const int16_t rate_ms) {
  ActionManager_IDL_setRecordingRate helper;
  helper.init(rate_ms);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","void ActionManager_IDL::setRecordingRate(const int16_t rate_ms)");
  }
  yarp().write(helper,helper);
}

bool ActionManager_IDL::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  yarp::os::ConstString tag = reader.readTag();
  bool direct = (tag=="__direct__");
  if (direct) tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
    if (tag == "play") {
      std::string name;
      double speed;
      bool sync;
      if (!reader.readString(name)) {
        reader.fail();
        return false;
      }
      if (!reader.readDouble(speed)) {
        speed = 0.5;
      }
      if (!reader.readBool(sync)) {
        sync = 1;
      }
      bool _return;
      _return = play(name,speed,sync);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "playFile") {
      std::string filename;
      double speed;
      bool sync;
      if (!reader.readString(filename)) {
        reader.fail();
        return false;
      }
      if (!reader.readDouble(speed)) {
        speed = 0.5;
      }
      if (!reader.readBool(sync)) {
        sync = 1;
      }
      bool _return;
      _return = playFile(filename,speed,sync);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "idleParts") {
      std::string parts;
      if (!reader.readString(parts)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = idleParts(parts);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "activeParts") {
      std::string parts;
      if (!reader.readString(parts)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = activeParts(parts);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "record") {
      std::string name;
      std::string parts;
      double timeout;
      if (!reader.readString(name)) {
        reader.fail();
        return false;
      }
      if (!reader.readString(parts)) {
        reader.fail();
        return false;
      }
      if (!reader.readDouble(timeout)) {
        timeout = 0;
      }
      bool _return;
      _return = record(name,parts,timeout);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "stopRecording") {
      bool _return;
      _return = stopRecording();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "stopPlaying") {
      bool _return;
      _return = stopPlaying();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "getAvailableActions") {
      std::vector<std::string>  _return;
      _return = getAvailableActions();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        {
          if (!writer.writeListBegin(BOTTLE_TAG_STRING, static_cast<uint32_t>(_return.size()))) return false;
          std::vector<std::string> ::iterator _iter5;
          for (_iter5 = _return.begin(); _iter5 != _return.end(); ++_iter5)
          {
            if (!writer.writeString((*_iter5))) return false;
          }
          if (!writer.writeListEnd()) return false;
        }
      }
      reader.accept();
      return true;
    }
    if (tag == "setRecordingRate") {
      int16_t rate_ms;
      if (!reader.readI16(rate_ms)) {
        reader.fail();
        return false;
      }
      setRecordingRate(rate_ms);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(0)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "help") {
      std::string functionName;
      if (!reader.readString(functionName)) {
        functionName = "--all";
      }
      std::vector<std::string> _return=help(functionName);
      yarp::os::idl::WireWriter writer(reader);
        if (!writer.isNull()) {
          if (!writer.writeListHeader(2)) return false;
          if (!writer.writeTag("many",1, 0)) return false;
          if (!writer.writeListBegin(BOTTLE_TAG_INT, static_cast<uint32_t>(_return.size()))) return false;
          std::vector<std::string> ::iterator _iterHelp;
          for (_iterHelp = _return.begin(); _iterHelp != _return.end(); ++_iterHelp)
          {
            if (!writer.writeString(*_iterHelp)) return false;
           }
          if (!writer.writeListEnd()) return false;
        }
      reader.accept();
      return true;
    }
    if (reader.noMore()) { reader.fail(); return false; }
    yarp::os::ConstString next_tag = reader.readTag();
    if (next_tag=="") break;
    tag = tag + "_" + next_tag;
  }
  return false;
}

std::vector<std::string> ActionManager_IDL::help(const std::string& functionName) {
  bool showAll=(functionName=="--all");
  std::vector<std::string> helpString;
  if(showAll) {
    helpString.push_back("*** Available commands:");
    helpString.push_back("play");
    helpString.push_back("playFile");
    helpString.push_back("idleParts");
    helpString.push_back("activeParts");
    helpString.push_back("record");
    helpString.push_back("stopRecording");
    helpString.push_back("stopPlaying");
    helpString.push_back("getAvailableActions");
    helpString.push_back("setRecordingRate");
    helpString.push_back("help");
  }
  else {
    if (functionName=="play") {
      helpString.push_back("bool play(const std::string& name, const double speed = 0.5, const bool sync = 1) ");
      helpString.push_back("play a recorded action using robot or simulator ");
    }
    if (functionName=="playFile") {
      helpString.push_back("bool playFile(const std::string& filename, const double speed = 0.5, const bool sync = 1) ");
      helpString.push_back("play a recorded action from a file using robot or simulator ");
    }
    if (functionName=="idleParts") {
      helpString.push_back("bool idleParts(const std::string& parts) ");
      helpString.push_back("idle parts: put the corresponding joint in idle mode ");
      helpString.push_back("@return true on success ");
    }
    if (functionName=="activeParts") {
      helpString.push_back("bool activeParts(const std::string& parts) ");
      helpString.push_back("activate parts: put the corresponding joint in controlled mode ");
      helpString.push_back("@return true on success ");
    }
    if (functionName=="record") {
      helpString.push_back("bool record(const std::string& name, const std::string& parts, const double timeout = 0) ");
      helpString.push_back("record a new action ");
      helpString.push_back("@return true on success ");
    }
    if (functionName=="stopRecording") {
      helpString.push_back("bool stopRecording() ");
      helpString.push_back("stop recording the action ");
      helpString.push_back("@return true on success ");
    }
    if (functionName=="stopPlaying") {
      helpString.push_back("bool stopPlaying() ");
      helpString.push_back("stop playing the action ");
      helpString.push_back("@return true on success ");
    }
    if (functionName=="getAvailableActions") {
      helpString.push_back("std::vector<std::string>  getAvailableActions() ");
      helpString.push_back("get the available actions ");
      helpString.push_back("@return a list of the available language ");
    }
    if (functionName=="setRecordingRate") {
      helpString.push_back("void setRecordingRate(const int16_t rate_ms) ");
      helpString.push_back("set the recording sample rate ");
      helpString.push_back("@param rate is the thread recoding rate in ms ");
    }
    if (functionName=="help") {
      helpString.push_back("std::vector<std::string> help(const std::string& functionName=\"--all\")");
      helpString.push_back("Return list of available commands, or help message for a specific function");
      helpString.push_back("@param functionName name of command for which to get a detailed description. If none or '--all' is provided, print list of available commands");
      helpString.push_back("@return list of strings (one string per line)");
    }
  }
  if ( helpString.empty()) helpString.push_back("Command not found");
  return helpString;
}


