#ifndef USAGECHECK_H_
#define USAGECHECK_H_

#include <fstream>
#include <iostream>
#include <stdlib.h>

using namespace std; 

void check_arguments(int argc, char* argv[]);
void check_files(ifstream& in_file, string& in_name, ofstream& out_file, string& out_name);

#endif /* USAGECHECK_H_ */