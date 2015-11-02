#include <ros/ros.h>
#include <akrobat/akrobat_init.h>
#include <akrobat/Akrobat.h>

using namespace std;
using namespace ros;

//---------------------------------MAIN-----------------------------------//
int main(int argc, char** argv)
{ 
  init(argc, argv, "akrobat_main");
  Akrobat akrobat;
  akrobat.initAkrobat();

  Rate r_schleife(20);
  //WHILE-LOOP
  while(ok() && ON ){
	akrobat.runAkrobat();
	spinOnce();
	r_schleife.sleep();
  }//END WHILE
}//-----------------------------END Main---------------------------------//


  




    

