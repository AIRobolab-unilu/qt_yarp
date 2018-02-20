// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <faceViewer_IDL.h>
#include <yarp/os/idl/WireTypes.h>



class faceViewer_IDL_setEmotion : public yarp::os::Portable {
public:
  std::string emoname;
  int16_t speed;
  bool _return;
  void init(const std::string& emoname, const int16_t speed);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class faceViewer_IDL_setEmotionFromFile : public yarp::os::Portable {
public:
  std::string filename;
  int16_t speed;
  bool _return;
  void init(const std::string& filename, const int16_t speed);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

bool faceViewer_IDL_setEmotion::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("setEmotion",1,1)) return false;
  if (!writer.writeString(emoname)) return false;
  if (!writer.writeI16(speed)) return false;
  return true;
}

bool faceViewer_IDL_setEmotion::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void faceViewer_IDL_setEmotion::init(const std::string& emoname, const int16_t speed) {
  _return = false;
  this->emoname = emoname;
  this->speed = speed;
}

bool faceViewer_IDL_setEmotionFromFile::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("setEmotionFromFile",1,1)) return false;
  if (!writer.writeString(filename)) return false;
  if (!writer.writeI16(speed)) return false;
  return true;
}

bool faceViewer_IDL_setEmotionFromFile::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void faceViewer_IDL_setEmotionFromFile::init(const std::string& filename, const int16_t speed) {
  _return = false;
  this->filename = filename;
  this->speed = speed;
}

faceViewer_IDL::faceViewer_IDL() {
  yarp().setOwner(*this);
}
bool faceViewer_IDL::setEmotion(const std::string& emoname, const int16_t speed) {
  bool _return = false;
  faceViewer_IDL_setEmotion helper;
  helper.init(emoname,speed);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool faceViewer_IDL::setEmotion(const std::string& emoname, const int16_t speed)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool faceViewer_IDL::setEmotionFromFile(const std::string& filename, const int16_t speed) {
  bool _return = false;
  faceViewer_IDL_setEmotionFromFile helper;
  helper.init(filename,speed);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool faceViewer_IDL::setEmotionFromFile(const std::string& filename, const int16_t speed)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool faceViewer_IDL::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  yarp::os::ConstString tag = reader.readTag();
  bool direct = (tag=="__direct__");
  if (direct) tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
    if (tag == "setEmotion") {
      std::string emoname;
      int16_t speed;
      if (!reader.readString(emoname)) {
        reader.fail();
        return false;
      }
      if (!reader.readI16(speed)) {
        speed = 1;
      }
      bool _return;
      _return = setEmotion(emoname,speed);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "setEmotionFromFile") {
      std::string filename;
      int16_t speed;
      if (!reader.readString(filename)) {
        reader.fail();
        return false;
      }
      if (!reader.readI16(speed)) {
        speed = 1;
      }
      bool _return;
      _return = setEmotionFromFile(filename,speed);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
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

std::vector<std::string> faceViewer_IDL::help(const std::string& functionName) {
  bool showAll=(functionName=="--all");
  std::vector<std::string> helpString;
  if(showAll) {
    helpString.push_back("*** Available commands:");
    helpString.push_back("setEmotion");
    helpString.push_back("setEmotionFromFile");
    helpString.push_back("help");
  }
  else {
    if (functionName=="setEmotion") {
      helpString.push_back("bool setEmotion(const std::string& emoname, const int16_t speed = 1) ");
      helpString.push_back("set the cuddie defualt emotions ");
      helpString.push_back("@return true/false on success/failure ");
    }
    if (functionName=="setEmotionFromFile") {
      helpString.push_back("bool setEmotionFromFile(const std::string& filename, const int16_t speed = 1) ");
      helpString.push_back("set the cuddie emotion from a file ");
      helpString.push_back("@return true/false on success/failure ");
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


