#include <iostream>
#include "fleet_scheduler/conflict_resolver.hpp"
int main() {
  ConflictResolver resolver;
  resolver.reserve_zone("robot1",0,0);
  if(!resolver.check_conflict("robot2",.5,0) || resolver.check_conflict("robot2",1,0)
      || resolver.check_conflict("robot1",0,0)) {return 1;}
  resolver.release_zone("robot1");
  if(resolver.check_conflict("robot2",.5,0)) {return 1;}
  std::cout<<"目标区域占用、1m 边界与释放: PASS\n";
}
