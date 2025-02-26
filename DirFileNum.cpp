#include "header.h"

ThePathDirFile ObtainFileNum(string path, vector<string>& fileName) 
{
    ThePathDirFile retStr = { 0, 0 };
    struct _finddata64i32_t filefind;
    string curr = path + "\\*.*";
    int   done = 0;
    intptr_t handle;
    int fileNum = 0;
    int dirNum = 0;
    if ((handle = _findfirst(curr.c_str(), &filefind)) == -1) { return retStr; }
    while (!(done = _findnext(handle, &filefind)))
    {
        //printf("%s\n", filefind.name);
        if (!strcmp(filefind.name, "..")) {
            continue;
        }
        //for (i = 0; i < layer; i++)cout << "     ";
        if ((_A_SUBDIR == filefind.attrib)) //是目录
        {
            //printf("----------%s\n", filefind.name);
            //cout << filefind.name << "(dir)" << endl;
            curr = path + "\\" + filefind.name;
            dirNum += 1;
        }
        else//不是目录，是文件
        {
            //cout << path + "\\" + filefind.name << endl;
            fileName.push_back(filefind.name);
            fileNum += 1;
        }
    }
    _findclose(handle);
    retStr._theDirNum = dirNum;
    retStr._theFileNum = fileNum;

    return retStr;
}