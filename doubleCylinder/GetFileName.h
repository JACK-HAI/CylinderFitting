#pragma once
#include <tchar.h>
#include <wnnc.h>
#include <vector>
#include <windows.h>
#include <io.h>

using namespace std;
void GetFileFmat(LPCTSTR path, vector <wstring > &filesPathVector, TCHAR *fmat);

//获得绝对路径中的最后文件名name，不包括path
//形参name为存储文件名的数组首地址
void get_name_from_pathname(char * const ppathname, char* name);
//void THCAR2char(TCHAR* tchStr, char chRtn[MAX_PATH]);
void TcharToChar(const TCHAR * tchar, char * _char);