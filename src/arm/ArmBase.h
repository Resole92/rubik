#ifndef ARMBASE
#define ARMBASE

class ArmBase{

 public:
  ArmBase(ros::NodeHandle *);
  std::string getType();
 private:
 int identifier;
};
#endif