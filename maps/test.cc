#include <iostream>
#include <stdio.h>
using namespace std;

int main()
{
   FILE *file = fopen("/home/shu/maps/tutorial.yaml", "r");
   if (!file) {
       cout << "Unable to open file";
   }
   else {
       cout << "Opened the file succesfully";
   }
   return 0;
}
