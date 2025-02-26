#ifndef __DIRFILENUM_HPP
#define __DIRFILENUM_HPP

#include <io.h>
#include <string>
#include <iostream>
#include <vector>
using namespace std;

struct ThePathDirFile 
{
    int _theFileNum;
    int _theDirNum;
};

ThePathDirFile ObtainFileNum(string path, vector<string>& fileName);

#endif

